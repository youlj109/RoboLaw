"""Actual pre-start provenance for the isolated additional-mustard collector."""
import json
from pathlib import Path
from .task_collection_evidence import TaskRunProvenance,TASK_CODE,require,json_new
from .mustard_family_preparation import ROOT,sha

FAMILY_CODE=tuple(sorted(set(TASK_CODE)|{
    'scripts/extensions/replicator/mustard_family_collect.py',
    *['kitchen_safety/extensions/replicator/'+name+'.py' for name in ('mustard_family_controller',
        'mustard_family_environment','mustard_family_background','mustard_family_preparation',
        'mustard_family_collision','mustard_family_recorder','mustard_family_evidence',
        'mustard_family_source_support','mustard_family_curobo')],
    *['scripts/extensions/replicator/'+name+'.py' for name in ('audit_initial_robot_geometry',
        'audit_fixed_base','audit_source_layout_geometry','search_source_layout_candidates','variant_geometry')],
    'kitchen_safety/grasp_geometry.py','kitchen_safety/placement_geometry.py',
}))


class MustardFamilyProvenance(TaskRunProvenance):
    def __init__(self,controller_path,controller,prepared):
        layout=ROOT/prepared['evidence']['input_hashes']['layout']['path']
        super().__init__(layout,collector_path=ROOT/'scripts/extensions/replicator/mustard_family_collect.py',input_paths=[controller_path])
        for path in FAMILY_CODE:self.capture(path,'execution_source',snapshot=True)
        for name,item in controller['inputs'].items():
            self.capture(item['path'],'family_cpu_preparation_'+name,snapshot=True,expected_sha256=item['sha256'])
        self.controller_sha256=sha(controller_path)
        self.relation_asset_aliases={}
        self._capture_relation_assets(prepared['evidence']['source_relations'])

    def _capture_relation_assets(self,audit):
        """Preserve every literal source-audit identity, including cache aliases."""
        require(bool(audit['assets_sha256']),'Missing exact original source asset inventory')
        for original,digest in audit['assets_sha256'].items():
            path=Path(original);path=ROOT/path if not path.is_absolute() else path
            require(sha(path)==digest,'Actual original source input differs before use: '+original)
            canonical=self.capture(path,'source_relation_asset',expected_sha256=digest)
            literal=Path(original)
            key=str(literal.relative_to(ROOT)) if literal.is_absolute() and literal.is_relative_to(ROOT) else str(literal)
            if key!=canonical:
                if key in self.entries:
                    require(self.entries[key]['initial_sha256']==digest,'Conflicting declared source alias '+key)
                else:
                    self.entries[key]=dict(self.entries[canonical],declared_path_alias=key,canonical_capture_key=canonical)
            self.relation_asset_aliases[key]=dict(canonical_capture_key=canonical,expected_sha256=digest)

    def bind_environment(self,definition):
        require(definition.resolved_task_layout==self.layout,'Actual family scene differs from declared original layout')
        info=definition.background_physics_provenance
        self.capture(info['source'],'source_kitchen',expected_sha256=info['source_sha256'])
        recipe=('kitchen_safety/extensions/replicator/mustard_family_background.py'
                if info['recipe']=='additional-mustard-fixed-refrigerator-housing-only-v1'
                else 'kitchen_safety/extensions/replicator/background_physics.py')
        require(info.get('recipe_source_path')==recipe,'Actual family recipe source identity differs')
        require(recipe in self.entries and self.entries[recipe]['initial_sha256']==info['recipe_sha256']
                and sha(ROOT/recipe)==info['recipe_sha256'],
                'Actual family recipe was not captured unchanged before initialization')
        self.capture(recipe,'execution_source',snapshot=True,expected_sha256=info['recipe_sha256'])
        self.capture(definition.scene.assets['kitchen'].usd_path,'composed_kitchen_overlay',expected_sha256=info['overlay_sha256'])
        source=definition.source_relation_audit
        self._capture_relation_assets(source)
        self.capture(source['source_yaml'],'source_task_yaml',snapshot=True,expected_sha256=source['source_yaml_sha256'])
        assembly=Path(definition.embodiment.scene_config.robot.spawn.usd_path)
        declared=[(path,digest) for path,digest in source['assets_sha256'].items() if Path(path).name==assembly.name]
        require(len(declared)==1 and sha(assembly)==declared[0][1],
                'Actual runtime assembled DROID differs from the original relation-audit assembly')
        assembly_key=self.capture(assembly,'composed_robot_on_stand',expected_sha256=declared[0][1])
        self.actual_robot_assembly=dict(runtime_spawn_path=str(assembly),canonical_capture_key=assembly_key,
            source_relation_asset_path=declared[0][0],expected_sha256=declared[0][1])
        self.background_physics=info;self.scene_bound=True

    def finish_task(self,directory,final_physics_step):
        require(self.scene_bound,'Actual family source scene was not bound before reset')
        report=self.check(final_physics_step=int(final_physics_step))
        report.update(task_id=self.task_id,source_layout=self.layout,background_physics=self.background_physics,
            controller_sha256=self.controller_sha256,required_execution_sources=list(FAMILY_CODE),
            original_source_relation_asset_aliases=self.relation_asset_aliases,
            actual_robot_assembly=self.actual_robot_assembly,
            initialization_scope='All FAMILY_CODE and declared CPU/control/source inputs captured before SimulationAppContext; actual overlay and stand bound before env construction/reset',
            scope='Actual selected runtime source and input start/end hashes with snapshots; no process-memory or external binary completeness claim.')
        json_new(Path(directory)/'task_collection_provenance.json',report)
        return report
