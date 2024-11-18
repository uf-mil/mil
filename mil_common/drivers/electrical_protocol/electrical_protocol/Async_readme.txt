Everything currently labelled async is unifinished
and not fully working.

The last issue I had was the simulated device not
being able to connect to the test, all async files are
commented out.

You can run the async test with the following command:
"rostest --text electrical_protocol async_simulated.test"

If you have any questions or confusion about the mess
I've made here feel free to reach out: marcinplaza@ufl.edu

ps: when starting you will most likely have to make
async_test_simulated_basic.py / async_driver.py executable
example: chmod +x async_test_simulated_basic.py (makes it executable)

some helpful sites:
axros examples
https://mil.ufl.edu/docs/reference/axros/examples.html

axros reference
https://mil.ufl.edu/docs/reference/axros/api.html#service

asyncio reference
https://docs.python.org/3/library/asyncio.html
