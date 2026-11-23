import io
import json
import types
import sys
import numpy as np
import pytest
from PIL import Image
from urc_core.onnx_detector import OnnxDetector


def test_cpu_decoder_reverses_letterboxing_and_suppresses_duplicates(tmp_path, monkeypatch):
    class FakeSession:
        def __init__(self, _model, providers, sess_options):
            assert providers == ['CPUExecutionProvider']
        def get_inputs(self):
            return [types.SimpleNamespace(name='images')]
        def run(self, _, inputs):
            tensor=inputs['images']
            assert tensor.shape==(1,3,640,640)
            return [np.array([[[320,322,420],[280,282,220],[100,100,60],[80,80,30], [.93,.84,.02],[.01,.01,.88]]],dtype=np.float32)]
    class Options:
        def add_session_config_entry(self, _name, _value):
            pass
    monkeypatch.setitem(sys.modules,'onnxruntime',types.SimpleNamespace(get_available_providers=lambda:['CPUExecutionProvider'],InferenceSession=FakeSession,SessionOptions=Options))
    weights=tmp_path/'model.onnx'
    weights.write_bytes(b'test model; inference is supplied by FakeSession')
    weights.with_suffix('.classes.json').write_text(json.dumps({'names':['bottle','hammer']}))
    detector=OnnxDetector(weights)
    image=io.BytesIO();Image.new('RGB',(640,360)).save(image,format='JPEG')
    results=detector.infer(image.getvalue())
    assert [kind for kind,_,_ in results]==['bottle','hammer']
    assert results[0][2]==(320.,140.,100.,80.)


def test_missing_yolo_model_and_manifest_have_actionable_errors(tmp_path):
    weights = tmp_path/'model.onnx'
    with pytest.raises(ValueError, match='YOLO model missing'):
        OnnxDetector(weights)
    weights.write_bytes(b'placeholder for missing-sidecar check')
    with pytest.raises(ValueError, match='class manifest missing'):
        OnnxDetector(weights)
