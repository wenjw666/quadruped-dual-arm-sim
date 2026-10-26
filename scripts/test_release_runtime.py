"""Exercise no-overwrite and complete-before-write runtime bundle installation."""
import io
import json
import tarfile
import tempfile
import unittest
from pathlib import Path
from release_runtime import FILES,digest,inspect,destination


class BundleTests(unittest.TestCase):
    def test_paths(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for name in ('../bad','cache/../../bad','/etc/passwd','workspace/output/v2/../../../bad','cache/..\\bad'):
                with self.assertRaises(ValueError):destination(name,root/'ws',root/'cache')

    def test_install_and_conflict(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);archive=root/'test.tar.gz';ws=root/'ws';cache=root/'cache'
            content={'workspace/'+n:b'test' for n in FILES};content['cache/mesh.obj']=b'mesh'
            meta={'schema':1,'version':'v0.2.0','files':{n:{'bytes':len(v),'sha256':digest(v)} for n,v in content.items()}}
            with tarfile.open(archive,'w:gz') as out:
                for n,v in {**content,'manifest.json':json.dumps(meta).encode()}.items():
                    info=tarfile.TarInfo(n);info.size=len(v);out.addfile(info,io.BytesIO(v))
            inspect(archive,ws,cache,True);inspect(archive,ws,cache,True)
            conflict=ws/FILES[-1];conflict.write_text('user data')
            (cache/'mesh.obj').unlink()
            with self.assertRaises(FileExistsError):inspect(archive,ws,cache,True)
            self.assertFalse((cache/'mesh.obj').exists());self.assertEqual(conflict.read_text(),'user data')
            with self.assertRaises(ValueError):inspect(archive,ws,cache,True,'0'*64)


if __name__=='__main__':unittest.main()
