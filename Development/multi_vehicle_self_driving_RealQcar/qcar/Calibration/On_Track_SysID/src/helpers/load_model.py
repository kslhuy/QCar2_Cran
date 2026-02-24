import yaml
import os
from .dotdict import DotDict
from .runtime import get_package_path

def get_dict(model_name, package_path=None):
    model, tire = model_name.split("_")
    package_root = get_package_path(package_path=package_path)
    file_path = os.path.join(package_root, "models", model, f"{model_name}.txt")
    with open(file_path, "rb") as f:
        params = yaml.load(f, Loader=yaml.Loader)
    
    return params

def get_dotdict(model_name, package_path=None):
    dict = get_dict(model_name, package_path=package_path)
    params = DotDict(dict)
    return params
