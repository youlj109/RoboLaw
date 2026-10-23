"""Full native mechanism recording, with unresolved USD mappings reported as failures."""
class MechanismObserver:
    """Read every kitchen articulation through native PhysX tensor views."""
    def __init__(self, stage, simulation_view, convert):
        from pxr import Usd, UsdPhysics
        self.convert = convert
        self.views = {}
        self.inventory = []
        self.issues = []
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if '/kitchen/' not in path or not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                continue
            view = simulation_view.create_articulation_view(path)
            if view.count != 1:
                self.issues.append({'path': path, 'count': int(view.count)})
                continue
            names = list(view.shared_metatype.dof_names)
            if not names:
                continue
            kinds = {}
            for child in Usd.PrimRange(prim):
                if child.IsA(UsdPhysics.RevoluteJoint):
                    kinds[child.GetName()] = 'revolute'
                elif child.IsA(UsdPhysics.PrismaticJoint):
                    kinds[child.GetName()] = 'prismatic'
            native_types = self.convert(view.get_dof_types())[0]
            # Native types also cover multi-DOF D6 joints (e.g. sink handle_joint:0).
            native_kinds = [{0: 'revolute', 1: 'prismatic'}.get(int(t), 'unknown')
                            for t in native_types]
            source_names = {child.GetName() for child in Usd.PrimRange(prim)
                            if child.IsA(UsdPhysics.Joint)}
            # PhysX splits a generic two-angular-DOF faucet joint into native
            # handle_temp_joint and handle_joint. Preserve both raw DOFs and
            # require the exact source generic joint with rotX/rotY limits.
            aliases = {}
            generic = [child for child in Usd.PrimRange(prim)
                       if child.GetTypeName() == 'PhysicsJoint'
                       and child.GetName() == 'handle_joint']
            if len(generic) == 1 and set(('handle_temp_joint','handle_joint')).issubset(names):
                from pxr import UsdPhysics
                source = generic[0]
                limits = [UsdPhysics.LimitAPI(source, axis) for axis in ('rotX','rotY')]
                if all(l and l.GetLowAttr().Get() < l.GetHighAttr().Get() for l in limits):
                    aliases['handle_temp_joint'] = 'handle_joint'
            unknown = [name for name, kind in zip(names, native_kinds)
                       if aliases.get(name,name.split(':')[0]) not in source_names
                       or kind == 'unknown' or (name in kinds and kinds[name] != kind)]
            if unknown:
                self.issues.append({'path': path, 'unmatched_dofs': unknown})
            self.views[path] = view
            print('MECHANISM_SOURCE_MAP',path,[(str(child.GetPath()),child.GetTypeName()) for child in Usd.PrimRange(prim) if child.IsA(UsdPhysics.Joint)],names,native_types.tolist(),flush=True)
            self.inventory.append({'path': path, 'joint_names': names,
                                   'joint_kinds': native_kinds, 'source_aliases': aliases, 'axis_mapping_note': 'Raw PhysX DOFs; generic faucet rotX/rotY expansion axes are not relabelled.'})
        if not self.views or self.issues:
            print('MECHANISM_MAPPING_UNRESOLVED',self.issues,flush=True)

    def read(self):
        return {path: {'q': self.convert(view.get_dof_positions())[0].tolist(),
                       'dq': self.convert(view.get_dof_velocities())[0].tolist()}
                for path, view in self.views.items()}

