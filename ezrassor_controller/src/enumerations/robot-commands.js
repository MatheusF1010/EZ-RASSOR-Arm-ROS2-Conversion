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
    STOPWHEELS: 'stop',
    DRIVEFORWARD: 'forward',
    DRIVEBACKWARD: 'backward',
    TURNLEFT: 'left',
    TURNRIGHT: 'right',
    UP: "RAISE",
    DOWN: "LOWER",
    STOP: "STOP",
    ROTATEOUTWARD: 1,
    ROTATEINWARD: -1,

    // Autonomous Functions
    DRIVE: 1,
    DIG: 2,
    DUMP: 4,
    SELFRIGHT: 8,
    FULLAUTONOMY: 16

    // Paver Arm
});

export {
    Robot, Operation
}