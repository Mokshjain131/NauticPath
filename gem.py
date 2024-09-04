import random
import math
from deap import base, creator, tools, algorithms

# Problem-specific parameters
num_ports = 10
coordinates = [(random.uniform(0, 100), random.uniform(0, 100)) for _ in range(num_ports)]

# Genetic algorithm setup
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()
toolbox.register("indices", random.sample, range(num_ports), num_ports)
toolbox.register("individual", tools.initIterate, creator.Individual, toolbox.indices)
toolbox.register("population", tools.initRepeat, list, toolbox.individual, n=100)

def distance(individual):
    total_distance = 0
    for i in range(num_ports - 1):
        p1, p2 = coordinates[individual[i]], coordinates[individual[i+1]]
        total_distance += math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    return (total_distance,)

toolbox.register("evaluate", distance)
toolbox.register("mate", tools.cxPartiallyMatched)
toolbox.register("mutate", tools.mutShuffleIndexes)
toolbox.register("select", tools.selTournament, tournsize=3)

# Run the genetic algorithm
population = toolbox.population()
hof = tools.HallOfFame(1)
stats = tools.Statistics(lambda ind: ind.fitness.values)
stats.register("avg", np.mean)
stats.register("min", np.min)
stats.register("max", np.max)

population, logbook = algorithms.eaSimple(population, toolbox, cxpb=0.7, mutpb=0.2, ngen=100, stats=stats, halloffame=hof)

# Best solution
best_individual = hof[0]
best_route = [coordinates[i] for i in best_individual]
best_distance = distance(best_individual)[0]
