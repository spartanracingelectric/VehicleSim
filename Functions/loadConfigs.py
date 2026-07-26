import importlib
import pkgutil
from types import SimpleNamespace

def loadConfigs(package_name: str) -> SimpleNamespace:
    package = importlib.import_module(package_name)
    configs = SimpleNamespace()

    for mod in pkgutil.iter_modules(package.__path__):
        if mod.name.startswith("_"):
            continue
        module = importlib.import_module(f"{package_name}.{mod.name}")
        setattr(configs, mod.name, getattr(module, mod.name))

    return configs