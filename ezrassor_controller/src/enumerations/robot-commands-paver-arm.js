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

    // Grabber open close
    GRABBERCLOSE: 'CLOSE',
    GRABBEROPEN: 'OPEN',

    // Stop all
    STOP: 'STOP',
    
    //SIMPLIFYING THE MOVEMENTS
    ROTATELEFT: "ROTATELEFT",
    ROTATERIGHT: "ROTATERIGHT",
    ROTATEUP: "ROTATEUP",
    ROTATEDOWN: "ROTATEDOWN",

    // Paver Arm Autonomy
    PICKUP: "PICKUP",
    PLACE: "PLACE",
    HOME: "HOME",

    // Paver Arm COmputer Vision Autonomy
    FULLAUTONOMY: 'AUTOMATIC',

});

export {
    Robot, Operation
}