"""Read-only observation of every registered native non-robot articulation."""


def mechanism_metadata(scene):
    return {
        name: {'joint_names': list(asset.joint_names), 'body_names': list(asset.body_names),
               'prim_path': asset.cfg.prim_path}
        for name, asset in sorted(scene.articulations.items()) if name != 'robot'
    }


def mechanism_snapshot(scene):
    """Every non-robot articulation, including target; no simulator writes."""
    result = {}
    for name, asset in sorted(scene.articulations.items()):
        if name == 'robot':
            continue
        result[name] = {'q': asset.data.joint_pos[0].detach().cpu().tolist(),
                        'dq': asset.data.joint_vel[0].detach().cpu().tolist()}
    return result
