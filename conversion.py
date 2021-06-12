import tensorrt as trt
## create logger
trt_logger = trt.Logger(trt.Logger.INFO)

builder = trt.Builder(trt_logger)
builder.max_batch_size = 1
builder.max_workspace_size = 1 << 32
builder.fp16_mode = True
builder.int8_mode = True
#network_creation_flag = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_PRECISION)
network_creation_flag = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)

network = builder.create_network(network_creation_flag)
profile = builder.create_optimization_profile()
config = builder.create_builder_config()

profile.set_shape("conv2d_input:0",(1,66,200,3),(1,66,200,3),(1,66,200,3))
config.add_optimization_profile(profile)

parser = trt.OnnxParser(network,trt_logger)

parser.parse_from_file("alan_bot_model_onnx.onnx")

engine = builder.build_engine(network,config)

print(engine)
print(engine.num_layers)

## save engine
with open("engine.plan", "wb") as f:
    f.write(engine.serialize())
