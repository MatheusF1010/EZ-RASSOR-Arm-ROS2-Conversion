const Robot = Object.freeze({
    JOINT1: Symbol('joint1'),
    JOINT2: Symbol('joint2'),
    JOINT3: Symbol('joint3'),
    JOINT4: Symbol('joint4'),
    JOINT5: Symbol('joint5'),
    CLAW: Symbol('claw'),
    AUTONOMY: Symbol('autonomy'),
    ALL: Symbol('all')
});

const Operation = Object.freeze({

    // Paver Arm

    // Arm right left
    ARMRIGHT: 'J12R',
    ARMLEFT: 'J12L',

    // Arm up down
    ARMUP: 'J23U',
    ARMDOWN: 'J23D', 

    // Claw up down
    CLAWUP: 'J34U',
    CLAWDOWN: 'J34D',

    // Claw right left
    CLAWRIGHT: 'J45R',
    CLAWLEFT: 'J45L',
    
    // Grabber rotate 
    GRABBERRIGHT: 'J56R',
    GRABBERLEFT: 'J56L',

    // Grabber open close
    GRABBERCLOSE: 'CLOSE',
    GRABBEROPEN: 'OPEN',

    // Stop all
    STOP: 'STOP', 

    // Paver Arm Autonomy
    PICKUP: {
        Joint1 : 2.5167,
        Joint2 : -1.3500,
        Joint3 : 1.1900,
        Joint4 : 0.1562,
        Joint5 : -0.1709,
    },
    PLACE: {
        Joint1 : 0.0000,
        Joint2 : -0.8505,
        Joint3 : 1.6489,
        Joint4 : -1.3017,
        Joint5 : 0.0000,
    },
    HOME: {
        Joint1 : 0.0000,
        Joint2 : 0.0000,
        Joint3 : 0.0000,
        Joint4 : 0.0000,
        Joint5 : 0.0000,
    },

    // Paver Arm COmputer Vision Autonomy
    FULLAUTONOMY: 'AUTOMATIC',

});

export {
    Robot, Operation
}