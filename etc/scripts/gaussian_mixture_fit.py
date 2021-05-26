#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
##########################################################################
#
#  gaussian_mixture_fit.py: fit gaussian mixture distribution to 1-D data
#
#  Copyright © 2021 Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
#
##########################################################################
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Library General Public License for more details.
#
#  Read the full text in the LICENSE.GPL file in the doc directory.

import argparse
import textwrap
import sys
from matplotlib import pyplot as plt
import numpy as np
from numpy import genfromtxt
from sklearn.mixture import GaussianMixture


def main():
  header = '''
###############################################################################
#                                                                             #
#   Fit gaussian mixture models to 1D data samples                            #
#                                                                             #
###############################################################################
'''
  parser = argparse.ArgumentParser(description=textwrap.dedent(header),
                                   formatter_class=argparse.RawTextHelpFormatter)
  parser.add_argument(
      '--csv',
      type=str,
      help='CSV file containing the data',
      required=True)
  parser.add_argument(
      '--delimeter',
      type=str,
      default=';',
      help='CSV delimeter (default: %(default)s)')
  parser.add_argument(
      '--min',
      type=int,
      default=1,
      help='minimum number of gaussians to fit (default: %(default)s)')
  parser.add_argument(
      '--max',
      type=int,
      default=10,
      help='maximum number of gaussians to fit (default: %(default)sv)')
  parser.add_argument(
      '--lower-bound', '-l',
      type=float,
      default=0,
      help='clip raw data to a lower bound')
  parser.add_argument(
      '--upper-bound', '-u',
      type=float,
      default=float('inf'),
      help='clip raw data to an upper bound')
  parser.add_argument(
      '--plot', '-p',
      action='store_true',
      help='plot the fit')
  parser.add_argument(
      '--repetitions', '-r',
      type=int,
      default=5,
      help='amount of repetitions for each number of gaussians (default: %(default)s)')
  parser.add_argument(
      '--error-criterion',
      default='BIC',
    choices=['BIC', 'AIC'],
      help='Akaike or Bayesian information criterion (default: %(default)s)')
  args = parser.parse_args(args=None if sys.argv[1:] else ['--help'])
  # validate inputs
  if args==None:
      parser.exit(1)

  X = genfromtxt(args.csv, delimiter=args.delimeter)
  X = np.fromiter([x for x in X
                   if x <= args.upper_bound and x >= args.lower_bound],
                     dtype=np.float64)
  sample_min=X.min()
  sample_max=X.max()
  X = np.reshape(X, (len(X), 1))


  # fit models:
  N = np.arange(args.min, args.max+1)
  M = args.repetitions
  models = [None for i in range(M*len(N))]

  for i in range(len(N)):
    for j in range(M):
      models[j+M*i] = GaussianMixture(N[i]).fit(X)

  # compute the AIC and the BICi
  AIC = [m.aic(X) for m in models]
  BIC = [m.bic(X) for m in models]

  if(args.error_criterion=="BIC"):
    M_best = models[np.argmin(BIC)]
  elif(args.error_criterion=="AIC"):
    M_best = models[np.argmin(AIC)]
  else:
    print("unknown error criterion")
    return
  weights = "-w"
  gauss_params = ""
  for i in M_best.weights_:
      weights += " " + str(i)
  for i in range(len(M_best.means_)):
      gauss_params+= " -g {:10.5f} {:10.5f}".format(M_best.means_[i][0],
                                                    M_best.covariances_[i][0][0])
  print(weights+gauss_params)
  if(args.plot):
    # Plot the results
    x = np.linspace(X.min(), X.max(), 1000)
    logprob = M_best.score_samples(x.reshape(-1, 1))
    responsibilities = M_best.predict_proba(x.reshape(-1, 1))
    pdf = np.exp(logprob)
    pdf_individual = responsibilities * pdf[:, np.newaxis]

    plt.hist(X, 100, density=True, alpha=0.4)
    plt.plot(x, pdf, '-k')
    plt.plot(x, pdf_individual, '--k')
    plt.text(0.04, 0.96, "Best-fit Mixture",
             ha='left', va='top')
    plt.show()



if __name__ == '__main__':
  main()
