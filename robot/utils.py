import time


def timeit(method):
    def timed(*args, **kwargs):
        start_time = time.time()  # Start the timer
        result = method(*args, **kwargs)  # Call the method
        end_time = time.time()  # End the timer
        execution_time = end_time - start_time  # Calculate execution time

        print(f"[{time.time()}] - {method.__name__} executed in {execution_time:.6f} seconds")
        return result
    return timed
