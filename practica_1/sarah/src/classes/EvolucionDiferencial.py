#!/usr/bin/env python

import random

"""
from pymoo.core.problem import Problem, ElementwiseProblem
from pymoo.problems.functional import FunctionalProblem
from pymoo.algorithms import DE
from pymoo.optimize import minimize
from pymoo.termination.default import DefaultSingleObjectiveTermination # Objetivo unico

# Problem: Definicion orientada a objetos que implementa un método que evalua un conjunto de soluciones.
# ElementwiseProblem: definicion orientada a objetos que implementa una funcion que evalúa una unica solución a la vez.
# FunctionalProblem: defina un problema mediante el uso de una funcion para cada objetivo y restricción.
"""

# https://towardsdatascience.com/pymoode-differential-evolution-in-python-78e4221e5cbe

class DifferentialEvolution(object):
    """
    pop_size (init, opcional): tamano de la poblacion
    sampling (muestreo, poblacion, o tipo matriz, opcional): estrategia de muestreo de pymooo
    variant (str, opcional): estrategia de evolucion diferencial. Debe ser una cadena con el 
                             formato "DE/selection/n/crossover". El valor predeterminado es
                             "DE/rand/1/bin"
    CR (float, opcional): parametro de cruce. Definido en el rango [0,1]. Para reforzar la 
                          mutacion, utilizar valores mas altos. Para controlar la velocidad de
                          convergencia, utilizar valores mas bajos
    F (iterable of float or float, opcional): factor de escala o parametro de mutacion. Definido
                                              en el rango (0,2). Para reforzar exploracion, usar
                                              valores mas altos, mientras que para la explotacion,
                                              usar valores mas bajos
    gamma (float, opcional): parametro de desviacion de fluctuacion. Debe estar en el rango (0,2). 
                             El valor predeterminado es 1e-4
    pm (mutation, opcional): operadores de mutacion de pymoo despues del cruce. El valor predeterminado
                             es None
    repair (callable or str): estrategia de reparacion de vectores mutantes fuera de los limites
                              del problema. El valor predeterminado es "bounce-back"
    survival (survival, opcional): la estrategia de supervivencia de pymooo. Debe considerarse en 
    """

    def __init__(self, F, CR, NP, Creator, *args, **kwargs):
        self.F = F     # Factor de peso
        self.CR = CR   # Constante crossover
        self.NP = NP   # Numero de padres

        self.Creator = Creator
        self.args = args
        self.kwargs = kwargs

        self.current = [Creator(*args, **kwargs) for i in range(NP)]
        self.candidates = [None for i in range(NP)]

        self.dimension = len(self.current[0])

    def recombine(self):
        self.candidates = []

        for x in self.current:
            a, b, c = random.sample(self.current, 3)
            y = self.Creator(*self.args, **self.kwargs)

            R = random.randint(0, self.dimension + 1)

            for i in range(self.dimension):
                if (random.uniform(0, 1) < self.CR) or (i == R):
                    y[i] = a[i] + self.F * (b[i] - c[i])
                else:
                    y[i] = x[i]

            self.candidates.append(y)

    def select(self):
        self.current = [max(current, candidate) for current, candidate in
                        zip(self.current, self.candidates)]

    def __getitem__(self, key):
        return self.current[key], self.candidates[key]

