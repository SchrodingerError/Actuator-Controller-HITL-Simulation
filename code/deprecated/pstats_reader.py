import cProfile
import pstats

# Load the profiling data from the file
stats = pstats.Stats('profile_results.txt')

# Print the top 10 functions by cumulative time
stats.sort_stats('cumulative').print_stats(25)