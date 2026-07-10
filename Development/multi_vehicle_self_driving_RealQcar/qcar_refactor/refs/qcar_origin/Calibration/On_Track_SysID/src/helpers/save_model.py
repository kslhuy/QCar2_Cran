import yaml
import os
from .runtime import get_logger, get_package_path

def save(model, overwrite_existing=True, verbose=False, package_path=None):

  logger = get_logger()
  package_root = get_package_path(package_path=package_path)
  file_path = os.path.join(package_root, "models", model['model_name'], model['model_name'] + "_" + model['tire_model'] + ".txt")
  if os.path.isfile(file_path):
    if (verbose): print("Model already exists")
    if overwrite_existing:
      if (verbose): print("Overwriting...")
    else:
      if (verbose): print("Not overwriting.")
      return 0

  try:
    model = model.to_dict()
  except:
    model = model

  os.makedirs(os.path.dirname(file_path), exist_ok=True)

  # Write data to the file
  with open(file_path, "w") as f:
      logger.loginfo(f"MODEL IS SAVED TO: {file_path}")
      yaml.dump(model, f, default_flow_style=False)
