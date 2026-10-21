"""Opt-in single-environment, three-view native Isaac RTX readout.

Install after reset and before camera warm-up. The original USD cameras, optics,
FrameViews and CameraData pose buffers remain authoritative. Only their render
products and RGB buffers are grouped. close() is terminal: call it after the
writer drains and before closing/rebuilding the environment, not to resume the
old three render products. Importing this module does not load Kit or CUDA.
"""
from copy import deepcopy
from types import MethodType
import time


DEFAULT_NAMES = ('external_camera', 'external_camera_2', 'wrist_camera')


def _compatible_cameras(scene, names):
    """Reject unsupported shared render settings before touching any product."""
    names = tuple(names)
    if int(scene.num_envs) != 1 or len(names) != 3 or len(set(names)) != 3:
        raise ValueError('Grouped camera prototype requires one environment and three distinct cameras')
    cameras = tuple(scene[name] for name in names)
    first = cameras[0]
    shared = ('width', 'height', 'data_types', 'background_color', 'update_period')
    paths = []
    for name, camera in zip(names, cameras):
        if getattr(camera, '_grouped_readout_owner', None) is not None:
            raise ValueError(f'{name}: a grouped readout is already installed')
        if not getattr(camera, '_is_initialized', False) or camera._render_data is None:
            raise ValueError(f'{name}: install grouped readout after camera initialization/reset')
        if int(camera.num_instances) != 1 or int(camera._view.count) != 1:
            raise ValueError(f'{name}: expected one camera view')
        if camera._renderer is not first._renderer:
            raise ValueError('Grouped views require the same native renderer instance/configuration')
        for field in shared:
            if getattr(camera.cfg, field) != getattr(first.cfg, field):
                raise ValueError(f'{name}: incompatible shared camera setting {field}')
        if camera.cfg.isp_cfg is not None:
            raise ValueError('Grouped prototype supports original RGB without a per-camera ISP')
        if list(camera.cfg.data_types) != ['rgb'] or camera.cfg.update_period != 0:
            raise ValueError('Grouped prototype requires RGB-only cameras with update_period=0')
        if (camera.cfg.width, camera.cfg.height) != (1280, 720):
            raise ValueError('Grouped prototype preserves the original 1280x720 cameras')
        if str(camera.device) != str(first.device) or not str(camera.device).startswith('cuda'):
            raise ValueError('Grouped prototype requires all camera buffers on the same CUDA device')
        spec = camera._render_data.spec
        if spec is None or len(spec.camera_prim_paths) != 1:
            raise ValueError(f'{name}: original render product must select exactly one USD camera')
        path = str(spec.camera_prim_paths[0])
        if list(map(str, camera._view.prim_paths)) != [path]:
            raise ValueError(f'{name}: original render product and FrameView paths differ')
        paths.append(path)
    if len(set(paths)) != 3:
        raise ValueError('Grouped views must reference three distinct original USD cameras')
    intrinsics = [camera._data.intrinsic_matrices.torch.detach().cpu().tolist()[0]
                  for camera in cameras]
    if any(matrix != intrinsics[0] for matrix in intrinsics[1:]):
        raise ValueError(
            'Grouped prototype does not support mixed intrinsics: the measured native tiled '
            'projection failed to preserve wrist K=666.7 alongside external K=500. '
            'Keep the original separate render products. Matching K alone does not establish '
            'that grouped material and lighting appearance is correct.')
    return cameras, tuple(paths)


def _grouped_update(camera, env_mask):
    camera._grouped_readout_owner.update(env_mask, camera)


class GroupedReadout:
    """Owner of one RP; normal Camera.update/data calls share each generation."""

    def __init__(self, cameras, names, renderer, render_data, camera_data, sim,
                 output_views, metadata):
        self.cameras, self.names = tuple(cameras), tuple(names)
        self.renderer, self.render_data, self.camera_data, self.sim = renderer, render_data, camera_data, sim
        self.output_views, self.metadata = output_views, metadata
        self._render_spec = render_data.spec
        self.metadata['render_product_history'] = [str(render_data.render_product.path)]
        self.metadata['refresh_events'] = []
        self.closed = False
        self._installed = False
        self._last_key = None
        self._old = []
        self.requests = self.grouped_read_count = self.empty_mask_requests = 0

    def install(self):
        if self._installed:
            raise RuntimeError('Grouped readout is already installed')
        self._old = [(camera._update_buffers_impl, camera._render_data, camera._data._output)
                     for camera in self.cameras]
        try:
            for camera, outputs in zip(self.cameras, self.output_views):
                camera._grouped_readout_owner = self
                camera._update_buffers_impl = MethodType(_grouped_update, camera)
                camera._render_data = None
                camera._data._output = outputs
            self._installed = True
        except BaseException:
            # No old RP has been destroyed yet, so installation can roll back.
            for camera, (method, old_data, outputs) in zip(self.cameras, self._old):
                camera._update_buffers_impl = method
                camera._render_data = old_data
                camera._data._output = outputs
                if getattr(camera, '_grouped_readout_owner', None) is self:
                    del camera._grouped_readout_owner
            raise
        # Stop all old products before the next app.update; otherwise they
        # would continue rendering alongside the new product.
        errors = []
        for camera, (_, old_data, _) in zip(self.cameras, self._old):
            try:
                camera._renderer.cleanup(old_data)
            except Exception as error:
                errors.append(repr(error))
        if errors:
            raise RuntimeError('Failed to retire original camera render products: ' + '; '.join(errors))

    def update(self, env_mask, requesting_camera):
        if self.closed:
            raise RuntimeError('Grouped camera readout is closed')
        if self.render_data is None:
            raise RuntimeError('Grouped render product is unavailable after a failed refresh')
        self.requests += 1
        key = (id(self.sim), int(self.sim.get_physics_step_count()), int(self.sim.render_generation))
        if key == self._last_key:
            return
        # Preserve the original sensor's empty-mask no-op. One check per new
        # requested generation replaces three identical one-env mask readbacks.
        if not requesting_camera._env_mask_has_any(env_mask):
            self.empty_mask_requests += 1
            return
        # Keep each original FrameView, camera K and camera convention. Calling
        # the native pose method refreshes measured metadata; it does not spawn
        # a camera or write any USD optical/mount attribute.
        for camera in self.cameras:
            camera._update_poses(env_mask=camera._ALL_ENV_MASK, frame_op=1)
        self.sim.render_context.render_into_camera(
            self.renderer, self.render_data, self.camera_data,
            self.sim.get_physics_step_count())
        self._last_key = key
        self.grouped_read_count += 1

    def refresh_render_product(self):
        """Recreate only the RP/annotator after an episode appearance change.

        Call on the simulation thread after the previous writer has drained,
        before the next original warm-up. USD cameras, per-camera pose/K state,
        GPU output allocations and their per-view aliases are retained. This
        resets product-local history; it does not claim to clear every global
        RTX cache or predict how many frames the unchanged warm-up will need.
        """
        if self.closed:
            raise RuntimeError('Grouped camera readout is closed')
        started = time.perf_counter()
        old_data = self.render_data
        event = {'old_render_product_path': (str(old_data.render_product.path) if old_data is not None else None),
                 'new_render_product_path': None, 'passed': False}
        self.metadata['refresh_events'].append(event)
        self.render_data = None
        self._last_key = None
        self.metadata['grouped_render_product_path'] = None
        new_data = None
        try:
            if old_data is not None:
                self.renderer.cleanup(old_data)
            # Keep the saved spec: native cleanup clears old_data.spec.
            new_data = self.renderer.create_render_data(self._render_spec)
            self.renderer.set_outputs(new_data, self.camera_data.output)
            self.render_data = new_data
            path = str(new_data.render_product.path)
            self.metadata['grouped_render_product_path'] = path
            self.metadata['render_product_history'].append(path)
            event.update(new_render_product_path=path, passed=True)
        except BaseException as error:
            event['error'] = f'{type(error).__name__}: {error}'
            if new_data is not None:
                try:
                    self.renderer.cleanup(new_data)
                except Exception as cleanup_error:
                    event['cleanup_error'] = repr(cleanup_error)
            raise
        finally:
            event['elapsed_seconds'] = time.perf_counter() - started
        return deepcopy(event)

    def report(self):
        return {**deepcopy(self.metadata), 'requests': self.requests,
                'grouped_read_count': self.grouped_read_count,
                'empty_mask_requests': self.empty_mask_requests, 'closed': self.closed}

    def close(self):
        if self.closed:
            return
        try:
            self.renderer.cleanup(self.render_data)
        finally:
            # Restoring the methods prevents a retained environment from holding
            # this owner. The destroyed old products are intentionally not revived.
            # Camera invalidation safely calls cleanup(None) during env.close().
            if self._installed:
                for camera, (method, _, outputs) in zip(self.cameras, self._old):
                    camera._update_buffers_impl = method
                    camera._render_data = None
                    camera._data._output = outputs
                    if getattr(camera, '_grouped_readout_owner', None) is self:
                        del camera._grouped_readout_owner
            self.closed = True


def install_grouped_readout(scene, names=DEFAULT_NAMES):
    """Replace three initialized RGB RPs with one native tiled RP; return owner.

    The real scene still contains one physical environment. num_instances=3 is
    only the render buffer view count; stage preparation is never repeated with
    a fictitious physical environment count. Warm-up remains the caller's normal
    original warm-up and must run after installation.
    """
    import warp as wp
    from isaaclab.renderers.camera_render_spec import CameraRenderSpec
    from isaaclab.sensors.camera.camera_data import CameraData
    from isaaclab.sim import SimulationContext
    from isaaclab.utils.warp import ProxyArray
    from isaaclab_physx.renderers.isaac_rtx_renderer import IsaacRtxRenderer

    names = tuple(names)
    cameras, paths = _compatible_cameras(scene, names)
    first, renderer = cameras[0], cameras[0]._renderer
    if not isinstance(renderer, IsaacRtxRenderer):
        raise ValueError('Grouped prototype supports the native Isaac RTX backend only')
    sim = SimulationContext.instance()
    if sim is None:
        raise RuntimeError('Grouped readout requires the initialized SimulationContext')
    cfg = deepcopy(first.cfg)
    spec = CameraRenderSpec(cfg=cfg, device=str(first.device), num_instances=3,
                            camera_prim_paths=paths, view_count=3,
                            camera_path_relative_to_env_0=paths[0].removeprefix('/World/envs/env_0/'))
    metadata = {
        'mode': 'native_three_view_grouped_render_product',
        'production_quota_eligible': False,
        'camera_names': list(names), 'camera_prim_paths': list(paths),
        'original_render_product_paths': [str(camera._render_data.render_product.path) for camera in cameras],
        'width': 1280, 'height': 720, 'view_count': 3, 'physical_num_envs': 1,
        'native_tile_grid': [2, 2], 'device': str(first.device),
        'usd_camera_optics_and_mounts_modified': False,
        'pose_authority': 'Each original Camera FrameView; original per-view K retained',
        'warmup_policy': 'Original caller warm-up; no frame reduction or relaxed convergence',
        'pixel_identity_claimed': False,
        'intrinsics': {name: camera._data.intrinsic_matrices.torch.detach().cpu().numpy()[0].tolist()
                       for name, camera in zip(names, cameras)},
    }
    render_data = owner = None
    try:
        render_data = renderer.create_render_data(spec)
        camera_data = CameraData.allocate(data_types=['rgb'], height=720, width=1280,
            num_views=3, device=str(first.device), supported_specs=renderer.supported_output_types())
        renderer.set_outputs(render_data, camera_data.output)
        output_views = []
        for index in range(3):
            aliases = {}
            for name, proxy in camera_data.output.items():
                source = proxy.warp
                aliases[name] = ProxyArray(wp.array(
                    ptr=source.ptr + index * source.strides[0], shape=(1,) + tuple(source.shape[1:]),
                    strides=source.strides, dtype=source.dtype, device=source.device, copy=False))
            output_views.append(aliases)
        metadata['grouped_render_product_path'] = str(render_data.render_product.path)
        owner = GroupedReadout(cameras, names, renderer, render_data, camera_data, sim,
                               output_views, metadata)
        owner.install()
        return owner
    except BaseException:
        if owner is not None:
            owner.close()
        elif render_data is not None:
            renderer.cleanup(render_data)
        raise
