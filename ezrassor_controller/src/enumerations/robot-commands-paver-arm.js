const Robot = Object.freeze({
    PAVERARM: Symbol('paverarm'),
    PAVERCLAW: Symbol('paverclaw'),
    // AUTONOMY: Symbol('autonomy'),
    ALL: Symbol('all')
});

const Operation = Object.freeze({

    // Paver Arm

    // Arm right left
    ARMRIGHT: 'joint12R',
    ARMLEFT: 'joint12L',
    ARMSTOP1: 'joint12S',

    // Arm up down
    ARMUP: 'joint23U',
    ARMDOWN: 'joint23D', 
    ARMSTOP2: 'joint23S',

    // Claw up down
    CLAWUP: 'joint34U',
    CLAWDOWN: 'joint34D',
    CLAWSTOP3: 'joint34S',

    // Claw right left
    CLAWRIGHT: 'joint45R',
    CLAWLEFT: 'joint45L',
    CLAWSTOP4: 'joint45S',
    
    // Grabber rotate 
    GRABBERRIGHT: 'joint56R',
    GRABBERLEFT: 'joint56L',
    GRABBERSTOP: 'joint56S',

    // Grabber open close
    GRABBERCLOSE: 'close',
    GRABBEROPEN: 'open',

    // Stop all
    STOP: 'stop', 

    // Paver Arm Autonomy
    PICKUP: '',
    PLACE: '',
    HOME: '',

});

export {
    Robot, Operation
}