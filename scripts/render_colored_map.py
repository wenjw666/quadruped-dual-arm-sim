"""Display only saved observed points: offline WebGL viewer, PNG and orbit video."""
import argparse,base64,json,math,subprocess
from pathlib import Path
import cv2
import numpy as np

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',default='/ws/output/mapping');a=p.parse_args();root=Path(a.root)
    data=np.load(root/'colored_map.npz');xyz=data['xyz'];rgb=data['rgb'];report=json.loads((root/'map_report.json').read_text())
    # Also support exports made by a mapper started before PCD export was added.
    packed=(rgb[:,0].astype(np.uint32)<<16)|(rgb[:,1].astype(np.uint32)<<8)|rgb[:,2].astype(np.uint32)
    pcd=np.column_stack((xyz,packed.view(np.float32))).astype('<f4')
    header=f'# Observed RGB-D; simulation-truth registration\nVERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n'
    (root/'colored_map.pcd').write_bytes(header.encode()+pcd.tobytes())
    center=(xyz.min(axis=0)+xyz.max(axis=0))/2
    encoded=base64.b64encode(np.column_stack((xyz,rgb/255)).astype('<f4').tobytes()).decode()
    html='''<!doctype html><meta charset="utf-8"><title>Observed factory RGB-D map</title>
<style>body{margin:0;background:#101820;color:#e4ecf0;font:15px system-ui}canvas{display:block;width:100vw;height:100vh}.info{position:absolute;left:24px;top:20px;background:#10202cdd;padding:18px;border-radius:8px;pointer-events:none}h2{margin:0 0 8px;font-size:21px}.controls{position:absolute;bottom:25px;left:24px}button{padding:10px;margin-right:8px;background:#284658;color:white;border:1px solid #6a9eaf;cursor:pointer}</style>
<canvas id="view"></canvas><div class="info"><h2>全场景彩色点云 / Observed RGB-D map</h2>POINTS points · VOXEL m voxels<br>实际相机 RGB + 深度；位姿使用仿真真值<br>未观测表面不补齐，不代表 SLAM 定位结果<br>拖动旋转 · Shift 拖动平移 · 滚轮缩放 · 双击恢复</div><div class="controls"><button id="top">俯视</button><button id="orbit">自动旋转</button></div>
<script>const c=document.getElementById('view'),g=c.getContext('webgl');if(!g)throw Error('WebGL unavailable');
const raw=atob('DATA'),buf=new ArrayBuffer(raw.length),bytes=new Uint8Array(buf);for(let i=0;i<raw.length;i++)bytes[i]=raw.charCodeAt(i);const data=new Float32Array(buf);
function shader(type,s){let v=g.createShader(type);g.shaderSource(v,s);g.compileShader(v);if(!g.getShaderParameter(v,g.COMPILE_STATUS))throw Error(g.getShaderInfoLog(v));return v}
let pr=g.createProgram();g.attachShader(pr,shader(g.VERTEX_SHADER,`attribute vec3 pos;attribute vec3 color;varying vec3 col;uniform vec3 center;uniform vec4 camera;uniform float aspect;void main(){vec3 p=pos-center;float a=camera.x,e=camera.y,d=camera.z;float h=cos(a)*p.x+sin(a)*p.y;float x=-sin(a)*p.x+cos(a)*p.y;float y=-sin(e)*h+cos(e)*p.z;float z=d-(cos(e)*h+sin(e)*p.z);gl_Position=vec4(x*2.2/aspect,y*2.2,1.002*z-.2002,z);gl_PointSize=camera.w;col=color;}`));
g.attachShader(pr,shader(g.FRAGMENT_SHADER,`precision mediump float;varying vec3 col;void main(){if(length(gl_PointCoord-vec2(.5))>.5)discard;gl_FragColor=vec4(col,1.);}`));g.linkProgram(pr);g.useProgram(pr);let b=g.createBuffer();g.bindBuffer(g.ARRAY_BUFFER,b);g.bufferData(g.ARRAY_BUFFER,data,g.STATIC_DRAW);for(const [name,offset]of [['pos',0],['color',12]]){let i=g.getAttribLocation(pr,name);g.enableVertexAttribArray(i);g.vertexAttribPointer(i,3,g.FLOAT,false,24,offset)}
const originalCenter=CENTER;let center=originalCenter.slice();g.enable(g.DEPTH_TEST);let az=-2.1,el=.9,dist=14,auto=false,drag=null;
c.onpointerdown=e=>{drag=[e.clientX,e.clientY];c.setPointerCapture(e.pointerId)};c.onpointerup=()=>drag=null;c.onpointermove=e=>{if(drag){let dx=e.clientX-drag[0],dy=e.clientY-drag[1];if(e.shiftKey){let s=dist/c.clientHeight;center[0]+=s*(dx*Math.sin(az)-dy*Math.sin(el)*Math.cos(az));center[1]+=s*(-dx*Math.cos(az)-dy*Math.sin(el)*Math.sin(az));center[2]+=s*dy*Math.cos(el)}else{az-=dx*.006;el=Math.max(.05,Math.min(1.57,el+dy*.006))}drag=[e.clientX,e.clientY]}};
c.onwheel=e=>{e.preventDefault();dist=Math.max(3,Math.min(35,dist*Math.exp(e.deltaY*.001)))};c.ondblclick=()=>{az=-2.1;el=.9;dist=14;center=originalCenter.slice()};document.getElementById('top').onclick=()=>el=1.57;document.getElementById('orbit').onclick=()=>auto=!auto;
function draw(){let d=window.devicePixelRatio||1;c.width=c.clientWidth*d;c.height=c.clientHeight*d;g.viewport(0,0,c.width,c.height);g.clearColor(.045,.07,.09,1);g.clear(g.COLOR_BUFFER_BIT|g.DEPTH_BUFFER_BIT);if(auto)az+=.003;g.uniform3fv(g.getUniformLocation(pr,'center'),center);g.uniform1f(g.getUniformLocation(pr,'aspect'),c.width/c.height);g.uniform4f(g.getUniformLocation(pr,'camera'),az,el,dist,2.4*d);g.drawArrays(g.POINTS,0,data.length/6);requestAnimationFrame(draw)}draw();</script>'''
    html=html.replace('POINTS',f'{len(xyz):,}',1).replace('VOXEL',str(report['voxel_m']),1).replace('DATA',encoded).replace('CENTER',json.dumps(center.tolist()))
    (root/'colored_map.html').write_text(html,encoding='utf-8')
    pos=xyz-center
    def render(az,el=.95,w=1280,h=720):
        ca,sa,ce,se=math.cos(az),math.sin(az),math.cos(el),math.sin(el)
        hor=ca*pos[:,0]+sa*pos[:,1];dep=14-(ce*hor+se*pos[:,2])
        u=(w/2+(-sa*pos[:,0]+ca*pos[:,1])*h*1.1/dep).astype(int)
        v=(h/2-(-se*hor+ce*pos[:,2])*h*1.1/dep).astype(int)
        valid=(u>=2)&(u<w-2)&(v>=2)&(v<h-2)&(dep>.1)
        ids=np.flatnonzero(valid);ids=ids[np.argsort(dep[ids])[::-1]]
        out=np.full((h,w,3),(23,18,12),np.uint8)
        for dx,dy in [(0,0),(1,0),(0,1),(1,1)]:out[v[ids]+dy,u[ids]+dx]=rgb[ids,::-1]
        cv2.putText(out,'OBSERVED RGB-D / ACCUMULATED MAP',(24,34),cv2.FONT_HERSHEY_SIMPLEX,.68,(230,240,240),1,cv2.LINE_AA)
        cv2.putText(out,f'{len(xyz):,} points | {report["voxel_m"]*100:.1f} cm voxels | simulation-truth registration',(24,h-24),cv2.FONT_HERSHEY_SIMPLEX,.52,(230,240,240),1,cv2.LINE_AA)
        return out
    cv2.imwrite(str(root/'colored_map.png'),render(-2.1,w=1920,h=1080))
    cv2.imwrite(str(root/'colored_map_top.png'),render(-math.pi/2,math.pi/2,w=1920,h=1080))
    encoder=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','bgr24','-s','1280x720','-r','10','-i','-','-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(root/'colored_map_orbit.mp4')],stdin=subprocess.PIPE)
    for i in range(180):encoder.stdin.write(render(-2.1+i*2*math.pi/180).tobytes())
    encoder.stdin.close()
    if encoder.wait():raise RuntimeError('ENCODING_FAILED')
    print(json.dumps(dict(points=len(xyz),viewer=str(root/'colored_map.html'))))
if __name__=='__main__':main()
