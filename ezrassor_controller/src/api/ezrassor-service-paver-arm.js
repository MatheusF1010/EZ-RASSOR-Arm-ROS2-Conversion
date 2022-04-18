import {Robot, Operation} from 'ezrassor-app/src/enumerations/robot-commands-paver-arm';
import HTTP from 'ezrassor-app/src/api/web-commands-paver-arm';

export default class EZRASSOR { 

    constructor(host, route = '') {
        this.host = host;
        this.route = route;
        this.setCoordinate(0, 0);
        this.allStop();
    }

    // Getters and Setters
    get host () {
        return this._host;
    }

    set host(value) {
        console.log(value);
        this._host = value;
    }

    get route() {
        return this._route;
    }

    set route(value) {
        if(value[0] === '/') {
            this._route = value.substring(1);
            return;
        }

        this._route = value;
    }

    get coordinate() {
        return this._coordinate;
    }

    setCoordinate(combinedCoordinate) {
       
        var coordinates = (typeof combinedCoordinate === "string") ? combinedCoordinate.trim() : '0';
        var split = coordinates.split(',');

        var x = split[0];        
        var y = '0';
        
        if (split.length == 2 && split[1] != '') {
            y = split[1];
        }

        this._coordinate = {
            x: parseInt(x),
            y: parseInt(y)
        }
    }

    // Build complete apiPath for HTTP requests
    get apiPath() {
        return 'http://' + this.host + '/' + this.route;
    } 

    // Return custom twist message
    get twistMsg() {
        return JSON.stringify(this._twistMsg);
    }

    // Update only the instruction needed
    updateTwistMsg(instruction) {
        this._twistMsg = instruction; 
    } 

    updateAutonomyTwistMsg(instruction) {
        if(instruction == Operation.DRIVE || instruction == Operation.FULLAUTONOMY) {
            this._twistMsg = {
                autonomous_toggles:instruction,
                target_coordinate:this.coordinate
            }
            return;
        }

        this._twistMsg = {autonomous_toggles:instruction};
    }
   
    // Stop all robot operations
    allStop = () => {
        this._twistMsg = { 
            joint_1_action: "STOP",
            joint_2_action: "STOP",
            joint_3_action: "STOP",
            joint_4_action: "STOP",
            joint_5_action: "STOP",
            paver_claw_action: "STOP",
            autonomous_toggles:0
        }

        HTTP.doPost(this.apiPath, this.twistMsg);
    }

    // Execute the corresponding robot command from the enumeration items passed in
    executeRobotCommand(part, operation) {
        // Needed when a stop override needs to occur
        if (part == Robot.ALL && operation == Operation.STOP) {
            this.allStop();
            return;
        }

        switch(part) {
            case Robot.JOINT1:
                this.updateTwistMsg({joint_1_action:operation});
                break;
            case Robot.JOINT2:
                this.updateTwistMsg({joint_2_action:operation});
                break;
            case Robot.JOINT3:
                this.updateTwistMsg({joint_3_action:operation});
                break;
            case Robot.JOINT4:
                this.updateTwistMsg({joint_4_action:operation});
                break;
            case Robot.JOINT5:
                this.updateTwistMsg({joint_5_action:operation});
                break;
            case Robot.CLAW:
                this.updateTwistMsg({claw_action:operation});
                break;
            case Robot.AUTONOMY:
                this.updateAutonomyTwistMsg(operation);
                break;
            default:
                console.log('Invalid robot part selected');
                return;
        }

        HTTP.doPost(this.apiPath, this.twistMsg);
    } 
}
