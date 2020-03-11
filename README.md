# FRC PID Tuner

## Overview

Allows deployment of any amount of arbitrary speed controllers.

Configs are loaded from JSON each enable session - Variables can be changed quickly without redeploying robot code.

## Example Subsystem (Drive)

```json
{
	"master": {
		"id": 2,
		"type": "FALCON",
		"gains": {
			"ff": 0.0,
			"f": 0.04781863509,
			"p": 0.01,
			"d": 0.0,
			"a": 15000.0,
			"v": 15000.0,
			"allowableError": 0.0
		},
		"isInverted": true,
		"isSensorInverted": true,
		"ramp": 0.1
	},
	"slaves": [
		{
			"id": 3,
			"type": "FALCON"
		},
		{
			"id": 12,
			"type": "FALCON",
			"isInverted": true
		},
		{
			"id": 13,
			"type": "FALCON",
			"isInverted": true
		}
	],
	"xboxId": 2,
	"aSetPoint": 0.0,
	"bSetPoint": 0.0,
	"xSetPoint": 0.0,
	"ySetPoint": 0.0
}
```

Now, the only code that needs to change is in [Robot.java](src/main/java/team8/tuner/Robot.java):

```java
public static final String kConfigFileName = "Drive";
```

Adding a new subsystem is as simple as placing it [here](src/main/deploy/config)
