from Development.multi_vehicle_self_driving_RealQcar.qcar_refactor.refs.qcar_origin.Calibration.id_contrl_analyse_racetrack.id_analyser.helpers.load_model import get_dotdict

model_name = "NUC1_linear" #name + tire model name

print(f"Loading model {model_name}...")
model = get_dotdict(model_name)
for name, value in model.items():
    print(str(name) + ": " + str(value))
