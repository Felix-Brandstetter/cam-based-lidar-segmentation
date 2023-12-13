import os
import numpy as np
import scipy as sp
import scipy.optimize


def create_empty_dir(dir):
  if not os.path.exists(dir):
    os.makedirs(dir)
  else:
    for f in os.listdir(dir):
        os.remove(os.path.join(dir, f))
  return dir