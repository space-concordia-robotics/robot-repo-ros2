# Running The Rover

This is a guide for starting the rover.

## Preparation

1. Ensure that the robot is in a position where it is safe if it begins to move
2. Ensure the robot has everything appropriately plugged in (if something is not plugged in, please talk to a member of the electrical team)
3. Ensure that all sensors are properly connected to the robot
4. Ask a member of the electrical team to turn on the robot

## Connecting

In order to connect to the robot, you need to do the following:

1. Connect the antenna to the PoE injector.
2. Connect the PoE injector to the base station WiFi AP
3. Connect your device to the base station WiFi AP either wirelessly or using an ethernet cable
4. ssh into the rover using `ssh nvidia@10.240.0.10`. It will prompt you for a password. If you do not know the password, please ask someone.

???+ note

    It is recommended that you add the jetson to your ssh config, to make it easier to ssh into.  
    Add the following to your `~/.ssh/config` (create the file & directory if it does not exist):
    ``` title="~/.ssh/config"
    Host rover
    HostName 10.240.0.10
    Port 22
    User nvidia
    Password [password here]
    ```
    If you have an ssh key, you can copy it over to the jetson using `ssh-copy-id nvidia@10.240.0.10`,
    and then replace `Password [password here]` with `IdentityFile ~/.ssh/id_ed25519`
    (or if you are not using an ed25519 ssh key, you can replace that with your ssh key)
    
    Once you have done this, instead of doing `ssh nvidia@10.240.0.10`, you can do `ssh rover`.

## Running

TODO
