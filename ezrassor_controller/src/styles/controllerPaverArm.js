import { StyleSheet } from 'react-native';

const ControllerStyle = StyleSheet.create({
  
  buttonLayoutContainer: {
    flex: 8,
    flexDirection: 'row',
    marginVertical: 10,
  },

  container: {
    flex: 1,
    backgroundColor: '#5D6061',
    alignItems: 'center',
    justifyContent: 'center',
  },
  
  dPadLeft: {
    flex: 1, 
    backgroundColor: '#3a3d3d', 
    borderRadius: 10, 
    marginHorizontal: 10, 
    elevation: 5, 
    justifyContent: 'center', 
    alignItems: 'center'
  },

  dPadRight: {
    flex: 1, 
    backgroundColor: '#3a3d3d', 
    borderRadius: 10,
    marginRight: 10, 
    elevation: 5, 
    justifyContent: 'center', 
    alignItems: 'center'
  },
  
  drumFunctionContainer: {
    flex: 6, 
    justifyContent: 'center', 
    marginHorizontal: 10,
    padding:10, 
    borderRadius: 10, 
    elevation: 3, 
    backgroundColor: '#2e3030'
  },

  wheelFunctionContainer: {
    flex: 3,
    marginLeft: 10,
    borderRadius: 10,
    elevation: 3,
    backgroundColor: '#2e3030',
  },

  headerContainer: {
    flex: 1,
    flexDirection: 'row',
    marginTop: 10,
    marginHorizontal: 10,
    elevation: 3,
    backgroundColor: '#2e3030',
    borderRadius: 10,
    padding: 10,
    justifyContent: 'center',
    alignItems: 'center'
  }, 

  image: {
    flex: 1,
    width: null,
    height: null,
    resizeMode: 'contain',
    paddingVertical:20,
  },

  textLarge: {
    fontFamily: 'NASA', 
    fontSize: 30, 
    color: '#fff'
  },

  textMedium: {
    fontFamily: 'NASA',
    flex: 4,
    fontSize: 25,
    color: '#fff',
    textAlign: 'center',
    textAlignVertical: 'center',
  },

  textSmall: {
    fontFamily: 'NASA', 
    fontSize: 22, 
    color: '#fff'
  },
  
  textTiny: {
    fontFamily: 'NASA', 
    fontSize: 12, 
    color: '#fff'
  },

  columnText: {
    color: '#fff',
    textAlign: 'center'
  },

  columnHyperlink: {
    color: 'cornflowerblue',
    textAlign: 'center'
  },

  columnHeader: {
    flex: 20,
    marginHorizontal: 15,
    justifyContent: 'center',
    alignItems: 'center' 
  },

  modalViewContainer: {
    borderRadius: 25,
    backgroundColor: '#5D6061',
  },

  modalButton: {
    flex: 1,
    backgroundColor: '#767676',
    borderRadius: 100,
    marginHorizontal: 15,
    justifyContent: 'center',
    alignItems: 'center',
    height: 100,
    elevation: 5,
  },

  upAndDownDPad: {
    flex: 1, 
    backgroundColor: '#3a3d3d', 
    borderRadius: 10, 
    margin: 10, 
    elevation: 5, 
    justifyContent: 'center', 
    alignItems: 'center'
  }, 

  ipInputBox: {
    height: 60, 
    fontSize: 50, 
    backgroundColor:'#2e3030', 
    borderColor: 'gray', 
    borderWidth: 1 , 
    color: '#fff', 
    textAlign: 'center',
    textAlignVertical: 'center', 
    fontFamily: 'NASA',
  },

  ipInputBoxMedium: {
    backgroundColor:'#2e3030', 
    borderColor: 'gray', 
    color: '#fff', 
    fontSize: 15, 
    textAlign: 'center',
    fontFamily: 'NASA',
    
  },

  rightSideRow: {
    flexDirection: 'row', 
    position: 'absolute', 
    right: 0 
  },

  buttonLayoutContainer: {
    flex: 8,
    flexDirection: 'row',
    marginVertical: 10,
  },

  container: {
    flex: 1,
    backgroundColor: '#5D6061',
    alignItems: 'center',
    justifyContent: 'center',
  },
  ArmContainer: {
    flex: 1,
    backgroundColor: '#5D6061'
  },
  
  dPadLeft: {
    flex: 1, 
    backgroundColor: '#3a3d3d', 
    borderRadius: 10, 
    marginHorizontal: 10, 
    elevation: 5, 
    justifyContent: 'center', 
    alignItems: 'center'
  },

  dPadRight: {
    flex: 1, 
    backgroundColor: '#3a3d3d', 
    borderRadius: 10,
    marginRight: 10, 
    elevation: 5, 
    justifyContent: 'center', 
    alignItems: 'center'
  },
  
  drumFunctionContainer: {
    flex: 6, 
    justifyContent: 'center', 
    marginHorizontal: 10,
    padding:10, 
    borderRadius: 10, 
    elevation: 3, 
    backgroundColor: '#2e3030'
  },

  wheelFunctionContainer: {
    flex: 3,
    marginLeft: 10,
    borderRadius: 10,
    elevation: 3,
    backgroundColor: '#2e3030' 
  },

  headerContainer: {
    flex: 1,
    flexDirection: 'row',
    marginTop: 10,
    marginHorizontal: 10,
    elevation: 3,
    backgroundColor: '#2e3030',
    borderRadius: 10,
    padding: 10,
    justifyContent: 'center',
    alignItems: 'center'
  }, 

  image: {
    flex: 1,
    width: null,
    height: null,
    resizeMode: 'contain',
    paddingVertical:20,
  },

  textLarge: {
    fontFamily: 'NASA', 
    fontSize: 30, 
    color: '#fff'
  },

  textMedium: {
    fontFamily: 'NASA',
    flex: 4,
    fontSize: 25,
    color: '#fff',
    textAlign: 'center',
    textAlignVertical: 'center',
  },

  textSmall: {
    fontFamily: 'NASA', 
    fontSize: 22, 
    color: '#fff'
  },

  mainButtonTextUp: {
    fontFamily: 'NASA', 
    fontSize: 15, 
    color: '#fff',
    textAlign: 'center',
    bottom: 5
  },

  mainButtonTextLeft: {
    fontFamily: 'NASA', 
    fontSize: 15, 
    color: '#fff',
    textAlign: 'center',
    bottom: 20
  },

  mainButtonTextRight: {
    fontFamily: 'NASA', 
    fontSize: 15, 
    color: '#fff',
    textAlign: 'center',
    bottom: 20
  },

  mainButtonTextDown: {
    fontFamily: 'NASA', 
    fontSize: 15, 
    color: '#fff',
    textAlign: 'center',
    top: 27
  },

  clawButtonTextDown: {
    fontFamily: 'NASA', 
    fontSize: 15, 
    color: '#fff',
    textAlign: 'center',
    top: 50
  },

  buttonTextUp: {
    fontFamily: 'NASA', 
    fontSize: 15, 
    color: '#fff',
    bottom: 15,
    textAlign: 'center'
  },

  buttonTextDown: {
    fontFamily: 'NASA', 
    fontSize: 15, 
    color: '#fff',
    top: 10,
    textAlign: 'center'
  },

  buttonTextCenter: {
    fontFamily: 'NASA', 
    fontSize: 22, 
    color: '#fff',
    textAlign: 'center'
  },

  buttonTextDownTiny: {
    fontFamily: 'NASA', 
    fontSize: 10, 
    color: '#fff',
  },

  buttonImage: {
    fontFamily: 'NASA', 
    color: '#fff',
    textAlign: 'center'
  },

  buttonImageMainDown: {
    fontFamily: 'NASA', 
    color: '#fff',
    textAlign: 'center',
    bottom: 20,
  },

  textSmallCenter: {
    textAlign: 'center',
    fontFamily: 'NASA', 
    fontSize: 22, 
    color: '#fff'
  },

  bugText: {
    textAlign: 'center',
    fontFamily: 'NASA', 
    fontSize: 22, 
    color: '#0E86D4'
  },
  
  devText: {
    fontFamily: 'NASA', 
    fontSize: 18, 
    color: '#fff',
    textAlign: 'center',
  },

  textTiny: {
    fontFamily: 'NASA', 
    fontSize: 12, 
    color: '#fff'
  },

  columnText: {
    color: '#fff',
    textAlign: 'center'
  },

  columnHyperlink: {
    color: 'cornflowerblue',
    textAlign: 'center'
  },

  columnHeader: {
    flex: 20,
    marginHorizontal: 15,
    justifyContent: 'center',
    alignItems: 'center' 
  },

  modalViewContainer: {
    borderRadius: 25,
    backgroundColor: '#5D6061',
  },

  modalButton: {
    flex: 1,
    backgroundColor: '#767676',
    borderRadius: 100,
    marginHorizontal: 15,
    justifyContent: 'center',
    alignItems: 'center',
    height: 100,
    elevation: 5,
  },

  upAndDownDPad: {
    flex: 1, 
    backgroundColor: '#3a3d3d', 
    borderRadius: 10, 
    margin: 10, 
    elevation: 5, 
    justifyContent: 'center', 
    alignItems: 'center'
  }, 

  ipInputBox: {
    height: 60, 
    fontSize: 50, 
    backgroundColor:'#2e3030', 
    borderColor: 'gray', 
    borderWidth: 1 , 
    color: '#fff', 
    textAlign: 'center',
    textAlignVertical: 'center', 
    fontFamily: 'NASA',
  },

  ipInputBoxMedium: {
    backgroundColor:'#2e3030', 
    borderColor: 'gray', 
    color: '#fff', 
    fontSize: 15, 
    textAlign: 'center',
    fontFamily: 'NASA',
    
  },

  rightSideRow: {
    flexDirection: 'row', 
    position: 'absolute', 
    right: 0 
  }

});

export default ControllerStyle;
