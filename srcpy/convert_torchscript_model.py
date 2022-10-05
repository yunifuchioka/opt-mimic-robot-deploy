import argparse
import torch

from model import ActorCriticNet

model_dir = "../models/"

model_input_dim = 22
# model_input_dim = 82
# model_input_dim = 106
model_hidden_dims = [128, 128]
model_output_dim = 8

parser = argparse.ArgumentParser()
parser.add_argument(
    "model_filename", help="Name of pytorch model. Exclude the .pt file extension")
args = parser.parse_args()

load_from_dir = model_dir + args.model_filename + ".pt"
save_to_dir = model_dir + args.model_filename + "_script.pt"

print("loading pytorch model at" + load_from_dir)
model = ActorCriticNet(model_input_dim, model_output_dim, model_hidden_dims)
model.load_state_dict(torch.load(
    load_from_dir, map_location=torch.device('cpu')))

example_input = torch.rand(1, model_input_dim)

model_script = torch.jit.trace(model, example_input)

model_script.save(model_dir + args.model_filename + "_script.pt")
print("saved torchscript model at " + save_to_dir)
