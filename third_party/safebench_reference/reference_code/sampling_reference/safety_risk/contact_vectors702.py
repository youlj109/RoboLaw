"""Read exactly three native coordinates without iterator exhaustion."""
import struct

AUDIT={'checked_vectors':0,'mismatches':0,'native_types':{}}

def contact_vec3_702(value):
    result=[float(value[0]),float(value[1]),float(value[2])]
    # Bounded exact comparison on actual native values, retained in metadata.
    if AUDIT['checked_vectors'] < 64:
        reference=[float(x) for x in value]
        name=type(value).__module__+'.'+type(value).__name__
        AUDIT['native_types'][name]=AUDIT['native_types'].get(name,0)+1
        AUDIT['checked_vectors']+=1
        if len(reference)!=3 or struct.pack('ddd',*reference)!=struct.pack('ddd',*result):
            AUDIT['mismatches']+=1
            raise ValueError('Native contact coordinate conversion differs from reference')
    return result
