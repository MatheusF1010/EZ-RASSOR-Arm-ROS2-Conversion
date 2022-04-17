const Robot = Object.freeze({
    WHEELS: Symbol('wheels'),
    FRONTDRUM: Symbol('frontdrum'),
    BACKDRUM: Symbol('backdrum'),
    FRONTARM: Symbol('frontarm'),
    BACKARM: Symbol('backarm'),
    AUTONOMY: Symbol('autonomy'),
    ALL: Symbol('all')
});

const Operation = Object.freeze({

    // Rover Wheels
    STOPWHEELS: 'stop',
    DRIVEFORWARD: 'forward',
    DRIVEBACKWARD: 'backward',
    TURNLEFT: 'left',
    TURNRIGHT: 'right',
    ROTATEOUTWARD: 1,
    ROTATEINWARD: -1,
    // LINEARX: "linear_x",
    // ANGULARZ: "angular_z",

    // Rover Drums
    // DRUMDUMP: "DUMP",
    // DRUMSTOP: "STOP",
    // DRUMDIG: "DIG",

    // Rover Arms
    UP: "RAISE",
    DOWN: "LOWER",
    STOP: "STOP",

    // Autonomous Functions
    DRIVE: 1, // AUTO_DRIVE
    DIG: 2, // AUTO_DIG
    DUMP: 4, // AUTO_DUMP
    SELFRIGHT: 8, // AUTO_DOCK  ??
    FULLAUTONOMY: 16 // FULL_AUTONOMY
    // STOP: "STOP",


});

export {
    Robot, Operation
}