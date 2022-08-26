
"use strict";

let NavSOL = require('./NavSOL.js');
let Ack = require('./Ack.js');
let NavSVINFO_SV = require('./NavSVINFO_SV.js');
let CfgMSG = require('./CfgMSG.js');
let EsfINS = require('./EsfINS.js');
let RxmRAWX = require('./RxmRAWX.js');
let MonGNSS = require('./MonGNSS.js');
let NavSTATUS = require('./NavSTATUS.js');
let NavTIMEUTC = require('./NavTIMEUTC.js');
let NavCLOCK = require('./NavCLOCK.js');
let CfgINF = require('./CfgINF.js');
let CfgPRT = require('./CfgPRT.js');
let CfgSBAS = require('./CfgSBAS.js');
let CfgRST = require('./CfgRST.js');
let CfgNAVX5 = require('./CfgNAVX5.js');
let MonHW = require('./MonHW.js');
let NavRELPOSNED = require('./NavRELPOSNED.js');
let CfgNMEA7 = require('./CfgNMEA7.js');
let NavPOSLLH = require('./NavPOSLLH.js');
let NavDGPS_SV = require('./NavDGPS_SV.js');
let CfgNAV5 = require('./CfgNAV5.js');
let CfgINF_Block = require('./CfgINF_Block.js');
let CfgUSB = require('./CfgUSB.js');
let NavSBAS_SV = require('./NavSBAS_SV.js');
let NavSAT_SV = require('./NavSAT_SV.js');
let CfgNMEA6 = require('./CfgNMEA6.js');
let CfgGNSS_Block = require('./CfgGNSS_Block.js');
let NavSAT = require('./NavSAT.js');
let CfgCFG = require('./CfgCFG.js');
let EsfRAW = require('./EsfRAW.js');
let Inf = require('./Inf.js');
let CfgDGNSS = require('./CfgDGNSS.js');
let EsfMEAS = require('./EsfMEAS.js');
let RxmRAW = require('./RxmRAW.js');
let RxmSVSI = require('./RxmSVSI.js');
let HnrPVT = require('./HnrPVT.js');
let UpdSOS = require('./UpdSOS.js');
let CfgTMODE3 = require('./CfgTMODE3.js');
let RxmRTCM = require('./RxmRTCM.js');
let CfgANT = require('./CfgANT.js');
let NavPVT = require('./NavPVT.js');
let NavTIMEGPS = require('./NavTIMEGPS.js');
let RxmSVSI_SV = require('./RxmSVSI_SV.js');
let NavATT = require('./NavATT.js');
let NavSBAS = require('./NavSBAS.js');
let NavSVINFO = require('./NavSVINFO.js');
let RxmRAW_SV = require('./RxmRAW_SV.js');
let CfgGNSS = require('./CfgGNSS.js');
let NavVELECEF = require('./NavVELECEF.js');
let NavPVT7 = require('./NavPVT7.js');
let NavDGPS = require('./NavDGPS.js');
let AidALM = require('./AidALM.js');
let EsfSTATUS_Sens = require('./EsfSTATUS_Sens.js');
let MonHW6 = require('./MonHW6.js');
let AidEPH = require('./AidEPH.js');
let CfgDAT = require('./CfgDAT.js');
let CfgRATE = require('./CfgRATE.js');
let RxmEPH = require('./RxmEPH.js');
let MonVER_Extension = require('./MonVER_Extension.js');
let MonVER = require('./MonVER.js');
let RxmSFRB = require('./RxmSFRB.js');
let RxmSFRBX = require('./RxmSFRBX.js');
let NavDOP = require('./NavDOP.js');
let RxmALM = require('./RxmALM.js');
let NavPOSECEF = require('./NavPOSECEF.js');
let NavVELNED = require('./NavVELNED.js');
let RxmRAWX_Meas = require('./RxmRAWX_Meas.js');
let EsfSTATUS = require('./EsfSTATUS.js');
let UpdSOS_Ack = require('./UpdSOS_Ack.js');
let EsfRAW_Block = require('./EsfRAW_Block.js');
let CfgNMEA = require('./CfgNMEA.js');
let NavSVIN = require('./NavSVIN.js');
let AidHUI = require('./AidHUI.js');
let CfgHNR = require('./CfgHNR.js');
let MgaGAL = require('./MgaGAL.js');
let TimTM2 = require('./TimTM2.js');

module.exports = {
  NavSOL: NavSOL,
  Ack: Ack,
  NavSVINFO_SV: NavSVINFO_SV,
  CfgMSG: CfgMSG,
  EsfINS: EsfINS,
  RxmRAWX: RxmRAWX,
  MonGNSS: MonGNSS,
  NavSTATUS: NavSTATUS,
  NavTIMEUTC: NavTIMEUTC,
  NavCLOCK: NavCLOCK,
  CfgINF: CfgINF,
  CfgPRT: CfgPRT,
  CfgSBAS: CfgSBAS,
  CfgRST: CfgRST,
  CfgNAVX5: CfgNAVX5,
  MonHW: MonHW,
  NavRELPOSNED: NavRELPOSNED,
  CfgNMEA7: CfgNMEA7,
  NavPOSLLH: NavPOSLLH,
  NavDGPS_SV: NavDGPS_SV,
  CfgNAV5: CfgNAV5,
  CfgINF_Block: CfgINF_Block,
  CfgUSB: CfgUSB,
  NavSBAS_SV: NavSBAS_SV,
  NavSAT_SV: NavSAT_SV,
  CfgNMEA6: CfgNMEA6,
  CfgGNSS_Block: CfgGNSS_Block,
  NavSAT: NavSAT,
  CfgCFG: CfgCFG,
  EsfRAW: EsfRAW,
  Inf: Inf,
  CfgDGNSS: CfgDGNSS,
  EsfMEAS: EsfMEAS,
  RxmRAW: RxmRAW,
  RxmSVSI: RxmSVSI,
  HnrPVT: HnrPVT,
  UpdSOS: UpdSOS,
  CfgTMODE3: CfgTMODE3,
  RxmRTCM: RxmRTCM,
  CfgANT: CfgANT,
  NavPVT: NavPVT,
  NavTIMEGPS: NavTIMEGPS,
  RxmSVSI_SV: RxmSVSI_SV,
  NavATT: NavATT,
  NavSBAS: NavSBAS,
  NavSVINFO: NavSVINFO,
  RxmRAW_SV: RxmRAW_SV,
  CfgGNSS: CfgGNSS,
  NavVELECEF: NavVELECEF,
  NavPVT7: NavPVT7,
  NavDGPS: NavDGPS,
  AidALM: AidALM,
  EsfSTATUS_Sens: EsfSTATUS_Sens,
  MonHW6: MonHW6,
  AidEPH: AidEPH,
  CfgDAT: CfgDAT,
  CfgRATE: CfgRATE,
  RxmEPH: RxmEPH,
  MonVER_Extension: MonVER_Extension,
  MonVER: MonVER,
  RxmSFRB: RxmSFRB,
  RxmSFRBX: RxmSFRBX,
  NavDOP: NavDOP,
  RxmALM: RxmALM,
  NavPOSECEF: NavPOSECEF,
  NavVELNED: NavVELNED,
  RxmRAWX_Meas: RxmRAWX_Meas,
  EsfSTATUS: EsfSTATUS,
  UpdSOS_Ack: UpdSOS_Ack,
  EsfRAW_Block: EsfRAW_Block,
  CfgNMEA: CfgNMEA,
  NavSVIN: NavSVIN,
  AidHUI: AidHUI,
  CfgHNR: CfgHNR,
  MgaGAL: MgaGAL,
  TimTM2: TimTM2,
};
