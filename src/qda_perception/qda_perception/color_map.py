"""Bounded voxel fusion of observed RGB-D points, never scene mesh sampling."""
import json
from pathlib import Path
import numpy as np


class ColorMap:
    def __init__(self, voxel=.035, limit=600000):
        self.voxel=voxel;self.limit=limit;self.cells={};self.frames=0;self.rejected=0

    def add(self, xyz, rgb):
        valid=np.isfinite(xyz).all(axis=1)
        xyz=np.asarray(xyz)[valid];rgb=np.asarray(rgb)[valid]
        keys=np.floor(xyz/self.voxel).astype(np.int32)
        # One contribution per voxel per frame avoids pixel-density bias.
        unique,index,inverse=np.unique(keys,axis=0,return_index=True,return_inverse=True)
        count=np.bincount(inverse)
        values=np.column_stack([np.bincount(inverse,weights=x)/count for x in np.column_stack((xyz,rgb)).T])
        for key,value in zip(unique,values):
            key=tuple(key)
            old=self.cells.get(key)
            if old is None:
                if len(self.cells)>=self.limit:self.rejected+=1;continue
                self.cells[key]=[value,1]
            else:
                n=min(old[1]+1,100);old[0]+=(value-old[0])/n;old[1]=n
        self.frames+=1

    def arrays(self):
        a=np.array([v[0] for v in self.cells.values()],dtype=np.float32).reshape(-1,6)
        return a[:,:3],np.clip(a[:,3:],0,255).astype(np.uint8)

    def save(self, directory):
        root=Path(directory);root.mkdir(parents=True,exist_ok=True)
        xyz,rgb=self.arrays()
        if not len(xyz):raise ValueError('EMPTY_COLOR_MAP')
        data=np.empty(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1')])
        for i,k in enumerate(('x','y','z')):data[k]=xyz[:,i]
        for i,k in enumerate(('red','green','blue')):data[k]=rgb[:,i]
        header='ply\nformat binary_little_endian 1.0\ncomment observed RGB-D; simulation-truth registration\nelement vertex '+str(len(xyz))+'\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n'
        (root/'colored_map.ply').write_bytes(header.encode()+data.tobytes())
        packed=(rgb[:,0].astype(np.uint32)<<16)|(rgb[:,1].astype(np.uint32)<<8)|rgb[:,2].astype(np.uint32)
        pcd=np.column_stack((xyz,packed.view(np.float32))).astype('<f4')
        header=f'# Observed RGB-D; simulation-truth registration\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n'
        (root/'colored_map.pcd').write_bytes(header.encode()+pcd.tobytes())
        np.savez_compressed(root/'colored_map.npz',xyz=xyz,rgb=rgb)
        report=dict(points=len(xyz),frames=self.frames,voxel_m=self.voxel,rejected_at_capacity=self.rejected,
                    bounds=[xyz.min(axis=0).tolist(),xyz.max(axis=0).tolist()],
                    source='observed head RGB-D pixels; dynamic robot/object masked by simulator segmentation',
                    registration='timestamp-matched MuJoCo camera pose (simulation truth), not SLAM',
                    hidden_surfaces='unobserved surfaces are not filled from model geometry')
        (root/'map_report.json').write_text(json.dumps(report,indent=2))
        return report
