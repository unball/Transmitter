import argparse
from controller import ProjectController
from regression import Idendify

parser = argparse.ArgumentParser()
parser.add_argument("-i", help="se i for adicionado na chamada, ele fará a identificacao",action="store_true")
args = parser.parse_args()

T = 0.002
ident = Idendify()

if args.i:
    ident.regression()
else:
    ident.readParm()
if ident.pA is not None:
    controller = ProjectController(gain=ident.pA[0],tau=ident.pA[1])
    print("-------------------------")
    print("Motor A: ")
    controller.definePIController()
    controller.discretize(T)
    print("-------------------------")
if ident.pB is not None:
    controller = ProjectController(gain=ident.pB[0],tau=ident.pB[1])
    print("-------------------------")
    print("Motor B: ")
    controller.definePIController()
    controller.discretize(T)
    print("-------------------------")