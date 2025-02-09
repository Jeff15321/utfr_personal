import onnx
onnx_model = onnx.load('/home/jeff1216/utfr_dv/src/perception/perception/models/yolov7-e6e.onnx')
onnx.checker.check_model(onnx_model)

print("Model is valid")
