import yaml
from trim_analysis import trim_f16_full

# Load parameters
with open("model/config/f16_params.yaml", "r") as f:
    params = yaml.safe_load(f)

x_trim, u_trim = trim_f16_full(params)

print("Trimmed state x:")
print(x_trim)
print("Trimmed input u:")
print(u_trim)
