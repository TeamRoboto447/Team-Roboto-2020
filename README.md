# Team Roboto (#447) 2020 Robot

## Java Profiling

Java profiling of the robot can be done using JMX options on the robot. JMX options are configured in the Java process that invokes the robot.

### Enable JMX on Robot

The commands used are:

```
-Dcom.sun.management.jmxremote=true -Dcom.sun.management.jmxremote.port=5801 -Dcom.sun.management.jmxremote.local.only=false -Dcom.sun.management.jmxremote.ssl=false -Dcom.sun.management.jmxremote.authenticate=false
```

Those enable remote JMX debugging on port 5801 without SSL and without requiring authentication.

The process that invokes Java is in `/home/lvuser/robotCommand`. That file is overwritten on each push of robot code. To address this, there are copies of the original robot command configuration and one that uses JMX available in the `/home/lvuser` directory along with shell scripts that enable and disable.

In case those are ever removed, the `enable-jmx` content looks like:

```
#!/bin/bash
rm robotCommand
cp robotCommand-jmx robotCommand
```

### Attach Profiler

For a profiler, you can use `jConsole` (ships with the JDK) or something like `visualVM` (available [here](https://visualvm.github.io/)).

Attach to the JMX process at `10.4.47.2:5801`.

With the profiler attached, you can watch memory, CPU, threads, etc. You can also take a memory snapshot and look at in memory objects.
