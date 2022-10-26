import React from 'react';
import Modal from "react-native-modal";
import FadeInView from "ezrassor-app/src/components/app/FadeInView";
import EZRASSOR from 'ezrassor-app/src/api/ezrassor-service-paver-arm' 
import ControllerStyle from 'ezrassor-app/src/styles/controllerPaverArm';
import {Robot, Operation} from 'ezrassor-app/src/enumerations/robot-commands-paver-arm';
import { Linking, Text, View, TouchableHighlight, TouchableOpacity, Image, StatusBar, KeyboardAvoidingView, TextInput} from 'react-native';
import { FontAwesome, MaterialCommunityIcons } from '@expo/vector-icons';
import * as Font  from 'expo-font';

export default class ControllerScreen extends React.Component {

  constructor(props) {
    super(props);
    this.state = {
      autonomyModalVisible: false,
      infoModalVisible: false,
      ipModal: false,
      xyModal: false,
      isLoading: true,
      control: 0,
      xy: '0,0',
      ip: '192.168.1.2:8080' 
    }; 

    this.EZRASSOR = new EZRASSOR(this.state.ip);
  }

  async componentDidMount () {
    await Font.loadAsync({
      'NASA': require('../../../assets/nasa.ttf'),
    });
    this.setState({ isLoading: false })
  }

  setAutonomyModalVisible(visible) {
    this.setState({ autonomyModalVisible: visible });
  }

  setInfoModalVisible(visible) {
    this.setState({ infoModalVisible: visible });
  }

  setIPModalVisible(visible){
    this.setState({ipModal: visible});
  }

  setXYModalVisible(visible){
    this.setState({xyModal:visible});
  }

  changeXY(combined){
    this.setState({xy:combined}, () => {
      this.EZRASSOR.setCoordinate(this.state.xy);
    });
  }

  changeIP(text){
    this.setState({ip:text}, () => {
      this.EZRASSOR.host = this.state.ip;
    });
  }

  // Update animation frame before processing click so that opacity can change on click 
  sendOperation(part, operation) {
    requestAnimationFrame(() => {
      this.EZRASSOR.executeRobotCommand(part, operation);
    });
  }

  render() {

    // Loading font
    if (this.state.isLoading) {
      return (
        <View style={{flex: 1, backgroundColor: '#5D6061'}}/>
      );
    }

    return ( 
      <View style={ControllerStyle.container}> 
        <StatusBar hidden />


        {/* Info Popup Modal*/}
        <Modal
          style={ControllerStyle.modalViewContainer}
          isVisible={this.state.infoModalVisible}
          onSwipeComplete={() => this.setInfoModalVisible(false)}
          swipeDirection='down'
          onRequestClose={() => this.setInfoModalVisible(!this.state.infoModalVisible)}>
    
            <View>
                <Text style={ControllerStyle.textLarge}>Activate Autonomous Arm Function(s)</Text>
            </View>
        </Modal>
        

        {/* Autonomy Popup Modal*/}
        <Modal
          style={ControllerStyle.modalViewContainer}
          isVisible={this.state.autonomyModalVisible}
          onSwipeComplete={() => this.setAutonomyModalVisible(false)}
          swipeDirection='down'
          onRequestClose={() => this.setAutonomyModalVisible(!this.state.autonomyModalVisible)}>
    
          <TouchableHighlight style={{ flex: 1, marginHorizontal: 15, justifyContent: 'center' }}>
            <View>
              <View style={{ flexDirection: 'row', marginVertical: 15, justifyContent: 'center' }}>
                <Text style={ControllerStyle.textLarge}>Activate Autonomous Arm Function(s)</Text>
              </View>
              <View style={{ flexDirection: 'row', marginVertical: 15, justifyContent: 'center' }}> 
                  <TouchableOpacity style={ControllerStyle.modalButton} onPress={() => {this.sendOperation(Robot.AUTONOMY, Operation.PICKUP)}}>
                    <Text style={ControllerStyle.textSmall}>Pick Up Paver</Text> 
                  </TouchableOpacity> 
                  <TouchableOpacity style={ControllerStyle.modalButton} onPress={() => {this.sendOperation(Robot.AUTONOMY, Operation.PLACE)}}>
                    <Text style={ControllerStyle.textSmall}>Place Paver</Text>
                  </TouchableOpacity>
                  <TouchableOpacity style={ControllerStyle.modalButton} onPress={() => {this.sendOperation(Robot.AUTONOMY, Operation.HOME)}}>
                    <Text style={ControllerStyle.textSmall}>Return Home</Text>
                  </TouchableOpacity>
              </View>
            </View>
          </TouchableHighlight>
        </Modal>
        
        <View style={ControllerStyle.headerContainer}>
        <TouchableOpacity style={{ flex: 1, padding: 1 }} onPress={() => this.setIPModalVisible(true)}>
            <FontAwesome name="search" size={30} color='#fff'/>
          </TouchableOpacity> 
          <TouchableOpacity style={{ flex: 1, padding: 3}} onPress={() => this.setInfoModalVisible(true)}>
            <FontAwesome name="info-circle" size={35} color='#fff'/>
          </TouchableOpacity>

          <Text style={ControllerStyle.textSmall}>EZ-RASSOR Robotic Arm Controller</Text>
          
          <TouchableOpacity style={{ flex: 1, padding: 3}} onPress={() => {this.sendOperation(Robot.ALL, Operation.STOP)}}>
            <FontAwesome name="ban" style={{marginLeft: "auto"}} size={35} color='#fff'/>
          </TouchableOpacity>

          <TouchableOpacity style={{ flex: 1}}onPress={() => { this.setAutonomyModalVisible(true)} }> 
            <MaterialCommunityIcons style={{marginLeft: "auto"}} name="robot" size={32} color='#fff'/>
          </TouchableOpacity>
        </View>

        <Modal
          style={ControllerStyle.modalViewContainer}
          isVisible={this.state.ipModal}
          onSwipeComplete={() => this.setIPModalVisible(false)}
          swipeDirection='down'
          onRequestClose={() => {this.setIPModalVisible(false)}}>
          <KeyboardAvoidingView
            paddingLeft={34}
            paddingRight={34}>
            <Text style={[ControllerStyle.textLarge, ControllerStyle.columnText]}>EZ-RASSOR Host to Control</Text>
            <TextInput
              style={ControllerStyle.ipInputBox}
              onChangeText={(text) => this.changeIP(text)}
              value={this.state.ip}
              marginVertical={20} />
          </KeyboardAvoidingView>
        </Modal>

        

        {/* Left D pad */}
        <View style={ControllerStyle.buttonLayoutContainer}>
        <View style={ControllerStyle.ArmContainer}> 
          <View style={{flex: 2 , flexDirection: 'column'}}>
            
            <View style={{flex: 2 , flexDirection: 'row'}}>
            <View style={ControllerStyle.wheelFunctionContainer}>
            <Text style={ControllerStyle.buttonTextCenter}>Base Joints</Text>

           <View style={ControllerStyle.upAndDownDPad} >
             <TouchableOpacity 
                 onPressIn={() => {this.sendOperation(Robot.JOINT2, Operation.ROTATEUP)}} 
                 onPressOut={() => {this.sendOperation(Robot.JOINT2, Operation.STOP)}}
                 hitSlop={{top: 20, bottom: 20, left: 50, right: 50}}
                 >  
               <FontAwesome tyle={ControllerStyle.buttonImage} name="chevron-up" size={50} color='#fff'/>
             </TouchableOpacity>
             </View>
             <View style={{flex: 2 , flexDirection: 'row'}}>
               <View style={ControllerStyle.dPadLeft}>
                 <TouchableOpacity
                     onPressIn={() => {this.sendOperation(Robot.JOINT1, Operation.ROTATELEFT)}}
                     onPressOut={() => {this.sendOperation(Robot.JOINT1, Operation.STOP)}}
                     hitSlop={{top: 30, bottom: 30, left: 50, right: 50}}
                     >
                   <FontAwesome tyle={ControllerStyle.buttonImage} name="chevron-left" size={50} color='#fff'/>
                 </TouchableOpacity>
               </View>
               <View style={ControllerStyle.dPadRight}>
                 <TouchableOpacity
                     onPressIn={() => {this.sendOperation(Robot.JOINT1, Operation.ROTATERIGHT)}}
                     onPressOut={() => {this.sendOperation(Robot.JOINT1, Operation.STOP)}}
                     hitSlop={{top: 30, bottom: 30, left: 50, right: 50}}
                     >
                   <FontAwesome tyle={ControllerStyle.buttonImage} name="chevron-right" size={50} color='#fff'/>
                 </TouchableOpacity>
               </View> 
               </View>
             <View style={ControllerStyle.upAndDownDPad} >
             <TouchableOpacity 
                 onPressIn={() => {this.sendOperation(Robot.JOINT2, Operation.ROTATEDOWN)}} 
                 onPressOut={() => {this.sendOperation(Robot.JOINT2, Operation.STOP)}}
                 hitSlop={{top: 20, bottom: 20, left: 50, right: 50}}
                 >  
               <FontAwesome tyle={ControllerStyle.buttonImage} name="chevron-down" size={50} color='#fff'/>
             </TouchableOpacity>
             </View>
             </View>
            {/* Right D pad OLD WORKING*/}
            {/* <View style={ControllerStyle.wheelFunctionContainer}>
           <View style={ControllerStyle.upAndDownDPad} >
             <TouchableOpacity 
                 onPressIn={() => {this.sendOperation(Robot.JOINT3, Operation.ROTATEUP)}} 
                 onPressOut={() => {this.sendOperation(Robot.JOINT3, Operation.STOP)}}
                 >  
               <FontAwesome name="chevron-up" size={50} color='#fff'/>
             </TouchableOpacity>
             </View>
             <View style={{flex: 2 , flexDirection: 'row'}}>
               <View style={ControllerStyle.dPadLeft}>
                 <TouchableOpacity
                     onPressIn={() => {this.sendOperation(Robot.JOINT4, Operation.ROTATELEFT)}}
                     onPressOut={() => {this.sendOperation(Robot.JOINT4, Operation.STOP)}}
                     >
                   <FontAwesome name="chevron-left" size={50} color='#fff'/>
                 </TouchableOpacity>
               </View>
               <View style={ControllerStyle.dPadRight}>
                 <TouchableOpacity
                     onPressIn={() => {this.sendOperation(Robot.JOINT4, Operation.ROTATERIGHT)}}
                     onPressOut={() => {this.sendOperation(Robot.JOINT4, Operation.STOP)}}
                     >
                   <FontAwesome name="chevron-right" size={50} color='#fff'/>
                 </TouchableOpacity>
               </View> 
               </View>
             <View style={ControllerStyle.upAndDownDPad} >
             <TouchableOpacity 
                 onPressIn={() => {this.sendOperation(Robot.JOINT3, Operation.ROTATEDOWN)}} 
                 onPressOut={() => {this.sendOperation(Robot.JOINT3, Operation.STOP)}}
                 >  
               <FontAwesome name="chevron-down" size={50} color='#fff'/>
             </TouchableOpacity>
             </View>
             </View> */}
        


            {/* NEW LAYOUT */}
            <View style={ControllerStyle.wheelFunctionContainer}>
            <Text style={ControllerStyle.buttonTextCenter}>Middle Joint</Text>
           <View style={ControllerStyle.upAndDownDPad} >
              <TouchableOpacity 
                  onPressIn={() => {this.sendOperation(Robot.JOINT3, Operation.ROTATEUP)}} 
                  onPressOut={() => {this.sendOperation(Robot.JOINT3, Operation.STOP)}}
                  hitSlop={{top: 20, bottom: 20, left: 50, right: 50}}
                  >  
                  <Text style={ControllerStyle.buttonTextUp}>Move Up</Text>
                <FontAwesome style={ControllerStyle.buttonImage} name="chevron-up" size={50} color='#fff'/>
              </TouchableOpacity>
             </View>
             <View style={ControllerStyle.upAndDownDPad} >
             <TouchableOpacity 
                 onPressIn={() => {this.sendOperation(Robot.JOINT3, Operation.ROTATEDOWN)}} 
                 onPressOut={() => {this.sendOperation(Robot.JOINT3, Operation.STOP)}}
                 hitSlop={{top: 20, bottom: 20, left: 50, right: 50}}
                 >  
               <FontAwesome style={ControllerStyle.buttonImage} name="chevron-down" size={50} color='#fff'/>
               <Text style={ControllerStyle.buttonTextDown}>Move Down</Text>
             </TouchableOpacity>
             </View>
             </View>

            <View style={ControllerStyle.wheelFunctionContainer}>
            <Text style={ControllerStyle.buttonTextCenter}>Upper Joint</Text>

            <View style={ControllerStyle.upAndDownDPad} >
              <TouchableOpacity
                    onPressIn={() => {this.sendOperation(Robot.JOINT4, Operation.ROTATERIGHT)}}
                    onPressOut={() => {this.sendOperation(Robot.JOINT4, Operation.STOP)}}
                    hitSlop={{top: 20, bottom: 20, left: 50, right: 50}}
                    >
                      <Text style={ControllerStyle.buttonTextUp}>Move Out</Text>
                  <FontAwesome style={ControllerStyle.buttonImage} name="chevron-up" size={50} color='#fff'/>
              </TouchableOpacity>
             </View>
             <View style={ControllerStyle.upAndDownDPad} >
                <TouchableOpacity
                    onPressIn={() => {this.sendOperation(Robot.JOINT4, Operation.ROTATELEFT)}}
                    onPressOut={() => {this.sendOperation(Robot.JOINT4, Operation.STOP)}}
                    hitSlop={{top: 20, bottom: 20, left: 50, right: 50}}
                    >
                  <FontAwesome style={ControllerStyle.buttonImage} name="chevron-down" size={50} color='#fff'/>
                  <Text style={ControllerStyle.buttonTextDown}>Move In</Text>
                </TouchableOpacity>
             </View>
             </View>



            <View style={ControllerStyle.wheelFunctionContainer}>
             <View style={{flex: 2 , flexDirection: 'column'}}>
             <Text style={ControllerStyle.buttonTextCenter}>Claw Joints</Text>

             <View style={ControllerStyle.upAndDownDPad} >
             <TouchableOpacity 
                 onPressIn={() => {this.sendOperation(Robot.JOINT5, Operation.ROTATERIGHT)}} 
                 onPressOut={() => {this.sendOperation(Robot.JOINT5, Operation.STOP)}}
                 hitSlop={{top: 20, bottom: 20, left: 50, right: 50}}
                 >  
               <FontAwesome name="rotate-right" size={50} color='#fff'/>
             </TouchableOpacity>
             </View>
               <View style={ControllerStyle.dPadLeft}>
                 <TouchableOpacity
                     onPressIn={() => {
                      if (this.grabber_flag == 0) {
                        this.sendOperation(Robot.CLAW, Operation.GRABBERCLOSE);
                        this.grabber_flag = 1;
                      } else {
                        this.sendOperation(Robot.CLAW, Operation.GRABBEROPEN);
                        this.grabber_flag = 0;
                      }
                      }}
                      hitSlop={{top: 20, bottom: 20, left: 50, right: 50}}
                      >
                   <Text style={ControllerStyle.textSmallCenter}>Open / Close Grabber</Text>
                 </TouchableOpacity>
               </View>
               <View style={ControllerStyle.upAndDownDPad} >
             <TouchableOpacity 
                 onPressIn={() => {this.sendOperation(Robot.JOINT5, Operation.ROTATELEFT)}} 
                 onPressOut={() => {this.sendOperation(Robot.JOINT5, Operation.STOP)}}
                 hitSlop={{top: 20, bottom: 20, left: 50, right: 50}}
                 >  
               <FontAwesome name="rotate-left" size={50} color='#fff'/>
             </TouchableOpacity>
             </View>
               </View>
             </View>
             </View>
           </View>
           </View>
        </View>
      </View>
    );
  }
} 
