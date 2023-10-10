![](./docs/images/banner.png)

# Overview
Pdoc auto-generates API documentation that follows your project's Python module hierarchy. To download pdoc input the next command in your terminal.
```{.bash .shell-prompt}
pip install pdoc
```

## Instructions
To generate the API documentation you need to follow this steps. First in your terminal type.
```{.bash .shell-prompt}
cd repos/stretch_body/docs/pdoc
```
Now that you are in the correct path you can start generating the documentation for Stretch Body, again in the terminal type.
```{.bash .shell-prompt}
firefox index.html
```
With this a tab in your firefox browser will open and it will be the documentation of the Stretch Body!

If you want to change something in the docs you can do that, open the file that you want to change, make the changes and save, then you can type in the same terminal.
```{.bash .shell-prompt}
./make.sh
```
Now refresh the page in your browser and you will see the changes there.

## Notes
If you want to hide something in the documentation then you can just type under the method a docstring to make it private, take this example:

```python
    RPC_SET_COMMAND = 1
    RPC_REPLY_COMMAND = 2
    """@private"""
    RPC_GET_STATUS = 3
```
If you generate this it will show the `RPC_SET_COMMAND = 1` and the `RPC_GET_STATUS = 3` but it will hide in the documentation the `RPC_REPLY_COMMAND = 2`. If you create a docstring with this format you will hide the constants, methods and even classes that you don't want them to appear, but be carefull because if you hide something, for example a method in a class, that you are using in different classes within the same code then you are going to hide every method. Take a look at this example from the stepper class to be more clear.
```python
    class StepperBase(Device):
        def push_load_test(self):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    class Stepper_Protocol_P3(StepperBase):
        def push_load_test(self):
        """Push a load test payload to the robot's hardware for testing
        """
        if not self.hw_valid:
            return
        payload = self.transport.get_empty_payload()
        payload[0] = self.RPC_LOAD_TEST_PUSH
        payload[1:] = self.load_test_payload
        self.transport.do_push_rpc_sync(payload, self.rpc_load_test_push_reply)
```
I want to hide from my documentation the `push_load_test` method from the `StepperBase` class but I have this same method in the `Stepper_Protocol_P3` class, if I don't want to hide one of them because has valuable information for the user then the solution for this is to write a docstring like the one in the example. With this we will only hide the method that we don't require to appear but leave the one that has information.