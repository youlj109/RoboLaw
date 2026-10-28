"""Outer position intervals for the actual float32 source placement tests.

Fixed yaw and fixed anchor positions only. XY predicates are intersections of
monotone half-lines and one absolute-distance interval. The original On size
test is deliberately omitted: this enlarges the accepted set. No geometric
clearance or source relation threshold is changed.
"""
import copy
import struct
import numpy as np
import torch
from isaaclab_arena.relations.relations import On, NextTo
from isaaclab_arena.relations.relation_loss_strategies import SIDE_CONFIGS, next_to_violations
from scripts.extensions.replicator.search_l_island_bowl_domain import source_position_domain


def ordered(value):
    bits=struct.unpack('I',struct.pack('f',value))[0]
    return (~bits)&0xffffffff if bits&0x80000000 else bits^0x80000000


def unordered(bits):
    raw=bits^0x80000000 if bits&0x80000000 else (~bits)&0xffffffff
    return struct.unpack('f',struct.pack('I',raw))[0]


def float32_outer_domain(scene,subject):
    exact=source_position_domain(scene,subject)
    low=np.asarray(exact['origin_xyz_lower']);high=np.asarray(exact['origin_xyz_upper'])
    child=scene.assets[subject];bbox=scene.bounds[child]
    if bbox.min_point.dtype!=torch.float32:raise ValueError('Float32 proof requires the observed source dtype')
    relations=[r for r in child.relations if isinstance(r,(On,NextTo))]
    # Finite anchors/bounds in [-16,16] plus containment On imply rejection
    # outside [-64,64]. The broad search boundary is independently checked.
    if not any(isinstance(r,On) and not r.overlap for r in relations):raise ValueError('Containment On is required')
    constants=[]
    for relation in relations:
        parent=scene.bounds[relation.parent]
        constants.extend(parent.min_point.flatten().tolist()+parent.max_point.flatten().tolist())
        if isinstance(relation,On):
            if relation.overlap:raise ValueError('Unsupported overlap relation')
            constants.extend([relation.edge_margin_m,relation.clearance_m])
        else:constants.extend([relation.distance_m,relation.tolerance_m])
    constants+=bbox.min_point.flatten().tolist()+bbox.max_point.flatten().tolist()
    if max(abs(v) for v in constants)>=16:raise ValueError('Source constants exceed proved broad interval')
    details=[]
    for axis in (0,1):
        def accept(value):
            x=bbox.min_point.new_tensor(value)
            for relation in relations:
                parent=scene.bounds[relation.parent]
                if isinstance(relation,On):
                    # Exactly the original translated/footprint arithmetic;
                    # anchor root is identity and its float32 zero sum is exact.
                    if not bool((bbox.min_point[0,axis]+x>=parent.min_point[0,axis]+relation.edge_margin_m)
                                & (bbox.max_point[0,axis]+x<=parent.max_point[0,axis]-relation.edge_margin_m)):
                        return False
                else:
                    cfg=SIDE_CONFIGS[relation.side]
                    if int(cfg.primary_axis)!=axis:continue
                    position=bbox.min_point.new_zeros((1,3));position[0,axis]=x
                    half,distance=next_to_violations(cfg,position,bbox,parent,relation.distance_m)
                    if half.item()>relation.tolerance_m or distance.item()>relation.tolerance_m:return False
            return True
        center=ordered(float((low[axis]+high[axis])/2))
        if not accept(unordered(center)) or accept(-64.) or accept(64.):raise ValueError('Invalid source interval bracket')
        left,right=ordered(-64.),center
        while right-left>1:
            mid=(left+right)//2
            if accept(unordered(mid)):right=mid
            else:left=mid
        first=right
        left,right=center,ordered(64.)
        while right-left>1:
            mid=(left+right)//2
            if accept(unordered(mid)):left=mid
            else:right=mid
        last=left
        a,b=unordered(first),unordered(last)
        prev,nxt=unordered(first-1),unordered(last+1)
        if accept(prev) or accept(nxt) or not accept(a) or not accept(b):raise ValueError('Float32 extremum verification failed')
        # Closed midpoint preimages cover both ties-to-even choices. Float32
        # values/midpoints are exact binary64 values at this magnitude.
        low[axis]=(prev+a)/2;high[axis]=(b+nxt)/2
        details.append(dict(axis=axis,first_accepted_float32=a,last_accepted_float32=b,
            rejected_previous_float32=prev,rejected_next_float32=nxt,
            real_input_outer_lower=low[axis],real_input_outer_upper=high[axis]))
    # On height arithmetic stays Python scalar64; three additions/comparisons
    # on magnitudes <64 have total absolute error <16*eps64*64. Outward rounding
    # is a proof enclosure, never an allowed change to an execution gate.
    z_enclosure=16*np.finfo(float).eps*64
    low[2]-=z_enclosure;high[2]+=z_enclosure
    result=copy.deepcopy(exact)
    result.update(origin_xyz_lower=low.tolist(),origin_xyz_upper=high.tolist(),
        lower_inclusive=[True]*3,upper_inclusive=[True]*3,
        analytic_real_arithmetic_domain=exact,float32_axis_endpoints=details,
        scalar64_z_outward_enclosure_m=z_enclosure,
        scope='Conservative closed outer interval of actual source float32 XY predicates and scalar64 height predicates, at fixed original yaw and fixed identity anchors. Original On size-fit check is ignored, enlarging the interval. Float32 bit-order bisection is complete because each source XY condition is a monotone half-line or absolute-distance interval; source code and finite brackets are bound by the certificate.')
    return result
