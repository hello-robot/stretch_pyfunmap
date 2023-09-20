from timeit import Timer, default_number
from tqdm import tqdm
import itertools
import gc


class ProgressTimer(Timer):
    """https://stackoverflow.com/a/70969696/4753010
    """

    def timeit(self, number=default_number):
        """Time 'number' executions of the main statement.
        To be precise, this executes the setup statement once, and
        then returns the time it takes to execute the main statement
        a number of times, as a float measured in seconds.  The
        argument is the number of times through the loop, defaulting
        to one million.  The main statement, the setup statement and
        the timer function to be used are passed to the constructor.
        """
        # wrap the iterator in tqdm
        it = tqdm(itertools.repeat(None, number), total=number)
        gcold = gc.isenabled()
        gc.disable()
        try:
            timing = self.inner(it, self.timer)
        finally:
            if gcold:
                gc.enable()
        # the tqdm bar sometimes doesn't flush on short timers, so print an empty line
        print()
        return timing