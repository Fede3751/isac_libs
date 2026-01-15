import time
import math
import random
import collections
from itertools import groupby
from enum import Enum, auto

from cachetools import Cache, FIFOCache, RRCache, LRUCache, LFUCache
from geometry_msgs.msg import Point


GAMMA = 1
NO_AGENTS = 1
LAMBDA_VALUE = 1


class CacheType(Enum):
    FIFO = 1
    RR = 2
    LRU = 3
    LFU = 4
    CIFO = 5

class BSUseCase(Enum):
    PROXIMITY = auto()
    COVERAGE = auto()
    BIDIRECT = auto()


class CacheExpiredError(KeyError):
    """Exception raised when a cache entry has expired."""
    pass

class KeyNotFoundError(KeyError):
    """Exception raised when a key is not found in the cache."""
    pass

class ExpiringCache:
    """A cache that expires items after a set time and limits the total entries."""
    def __init__(self, cache: Cache, expiration_time: float, maxsize: int, cache_device = None):
        self.cache = cache
        self.expiration_time = expiration_time  
        self.maxsize = maxsize
        self.cache_device = cache_device

    def __setitem__(self, key, value):
        """Add a new item to the cache."""

        # Before adding, check if expired entries are in the cache and remove one
        # first to create space
        self.cleanup(delete_excess=True)

        self.cache[key] = value

    def __getitem__(self, key):
        """Retrieve an item, checking if it has expired."""
        if key in self.cache:

            return self.cache[key]
        
            if time.time() - self.cache[key].misuration_ts < self.expiration_time:
                return self.cache[key]
            else:
                self.cache.pop(key)
                raise CacheExpiredError()
        else:
            raise KeyNotFoundError()

    def clear(self):
        """Clear the cache and timestamp tracking."""
        self.cache.clear()

    def __len__(self):
        return len(self.cache)

    def __repr__(self):
        """String representation of the cache contents and expiration time."""
        cache_contents = {key: value for key, value in self.cache.items()}
        return f"<ExpiringCache(cache={cache_contents}, expiration_time={self.expiration_time}s)>"
    
    def items(self):
        return self.cache.items()
    
    def cleanup(self, delete_excess = True, other_caches = None):

        # while True:

        #     deleted_once = False
                
        #     for key_expired in self.cache:
        #         if time.time() - self.cache[key_expired].misuration_ts >= self.expiration_time:
        #             deleted_once = True
        #             break

        #     if deleted_once:
        #         self.cache.pop(key_expired)
        #     else:
        #         break

        if delete_excess:
            while len(self.cache) > self.maxsize:
                self.cache.popitem()

    def has_item(self, key):

        #self.cleanup(delete_excess=False)

        if key in self.cache:
            return True
        
        return False




class CacheFactory:
    @staticmethod
    def create(typ: str, size: int, expiration_time: float, cache_device = None ) -> ExpiringCache:

        is_partitioned = False

        """Create a cache with a specific type and expiration time."""
        try:
            cache_type = CacheType[typ.upper()]
        except KeyError:
            raise ValueError(f'Invalid cache type: {typ}')

        if cache_type == CacheType.FIFO:
            cache = FIFOCache(maxsize=size)
        elif cache_type == CacheType.RR:
            cache = RRCache(maxsize=size)
        elif cache_type == CacheType.LRU:
            cache = LRUCache(maxsize=size)
        elif cache_type == CacheType.CIFO:
            cache = CIFOCache(maxsize=size)

        else:
            
            cache = LFUCacheTie(maxsize=size)

        return ExpiringCache(cache, expiration_time, maxsize = size)



class RandomCache(Cache):

    def __init__(self, maxsize):
    
        super().__init__(maxsize)


    def popitem(self):

        random_item = random.choice(list(self.items().keys()))
        return self.pop(random_item)





class CIFOCache(Cache):
    def __init__(self, maxsize, last_query = (0,0)):
        super().__init__(maxsize)
        self.__counter = collections.Counter()


    
    def __getitem__(self, key, cache_getitem=Cache.__getitem__):
        value = cache_getitem(self, key)
        
        if key in self:
            self.__counter[key] += 1

        return value



    def popitem(self):
        """Remove the entry closest to the cache's position."""
        
        to_remove = None

        min_strength = math.inf

        # Find the maximum values to normalize the eviction factors
        for key, value in self.items():
            
            strength = value.strength
            
            if strength < min_strength:
                min_strength = min_strength
                to_remove = key

        if to_remove is not None:
            self.__counter[to_remove] = 0
            return self.pop(to_remove)
        

    def __setitem__(self, key, value):

        super().__setitem__(key, value)

        if len(self) > self.maxsize:
                            
            self.popitem()



class LFUCacheTie(Cache):

    def __init__(self, maxsize, getsizeof=None):
        Cache.__init__(self, maxsize, getsizeof)
        self.__counter = collections.Counter()

    def __getitem__(self, key, cache_getitem=Cache.__getitem__):
        value = cache_getitem(self, key)
        if key in self:  # __missing__ may not store item
            self.__counter[key] -= 1
        return value

    def __setitem__(self, key, value, cache_setitem=Cache.__setitem__):
        cache_setitem(self, key, value)

        # for k in self.__counter.keys():
        #     self.__counter[k] += 1
            
        self.__counter[key] -= 1

    def __delitem__(self, key, cache_delitem=Cache.__delitem__):
        cache_delitem(self, key)
        del self.__counter[key]

    def popitem(self):
        """Remove and return the `(key, value)` pair least frequently used."""
        try:
            # Group by count, shuffle groups with the same count
            most_common = self.__counter.most_common()
            #most_common = [(k, v) for k, v in sorted(self.__counter.items(), key=lambda item: item[1])]

            #print(most_common)

            randomized_result = []
            for count, group in groupby(most_common, key=lambda x: x[1]):
                group = list(group)  # Convert group to list
                random.shuffle(group)  # Shuffle ties
                randomized_result.extend(group)

            key = randomized_result[0][0]
                
        except ValueError:
            raise KeyError("%s is empty" % type(self).__name__) from None
        else:
            return (key, self.pop(key))