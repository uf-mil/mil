def thread_lock(lock):
    '''Use an existing thread lock to thread-lock a function
    This prevents the function from being executed by multiple threads at once

    Example:
    import threading
    lock = threading.Lock()

    @thread_lock(lock)
    def my_function(a, b, c):
        print a, b, c
    '''

    def lock_thread(function_to_lock):
        '''thread_lock(function) -> locked function
        Thread locking decorator
            If you use this as a decorator for a function, it will apply a threading lock during the execution of that function,
            Which guarantees that no ROS callbacks can change the state of data while it is executing. This
                is critical to make sure that a new message being sent doesn't cause a weird serial interruption
        '''
        def locked_function(*args, **kwargs):
            # Get threading lock
            with lock:
                result = function_to_lock(*args, **kwargs)
            # Return, pretending the function hasn't changed at all
            return result
        # Return the function with locking added
        return locked_function

    return lock_thread