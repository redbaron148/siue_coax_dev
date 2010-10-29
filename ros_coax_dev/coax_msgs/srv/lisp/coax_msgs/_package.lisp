(defpackage coax_msgs-srv
  (:use cl
        roslisp-msg-protocol)
  (:export
   "COAXGETCONTROLPARAMETERS"
   "<COAXGETCONTROLPARAMETERS-REQUEST>"
   "<COAXGETCONTROLPARAMETERS-RESPONSE>"
   "COAX3DSETCONTROLMODE"
   "<COAX3DSETCONTROLMODE-REQUEST>"
   "<COAX3DSETCONTROLMODE-RESPONSE>"
   "COAXSETTRIMMODE"
   "<COAXSETTRIMMODE-REQUEST>"
   "<COAXSETTRIMMODE-RESPONSE>"
   "COAXREQUESTSTATE"
   "<COAXREQUESTSTATE-REQUEST>"
   "<COAXREQUESTSTATE-RESPONSE>"
   "COAXSETCONTROL"
   "<COAXSETCONTROL-REQUEST>"
   "<COAXSETCONTROL-RESPONSE>"
   "COAXCONFIGURECONTROL"
   "<COAXCONFIGURECONTROL-REQUEST>"
   "<COAXCONFIGURECONTROL-RESPONSE>"
   "COAXSETACKMODE"
   "<COAXSETACKMODE-REQUEST>"
   "<COAXSETACKMODE-RESPONSE>"
   "COAXGETTRIMMODE"
   "<COAXGETTRIMMODE-REQUEST>"
   "<COAXGETTRIMMODE-RESPONSE>"
   "COAXSETCONTROLPARAMETERS"
   "<COAXSETCONTROLPARAMETERS-REQUEST>"
   "<COAXSETCONTROLPARAMETERS-RESPONSE>"
   "COAXCONFIGURECOMM"
   "<COAXCONFIGURECOMM-REQUEST>"
   "<COAXCONFIGURECOMM-RESPONSE>"
   "COAXSETTIMEOUT"
   "<COAXSETTIMEOUT-REQUEST>"
   "<COAXSETTIMEOUT-RESPONSE>"
   "COAXGETVERSION"
   "<COAXGETVERSION-REQUEST>"
   "<COAXGETVERSION-RESPONSE>"
   "COAXSENDSTRING"
   "<COAXSENDSTRING-REQUEST>"
   "<COAXSENDSTRING-RESPONSE>"
   "COAXSETRAWCONTROL"
   "<COAXSETRAWCONTROL-REQUEST>"
   "<COAXSETRAWCONTROL-RESPONSE>"
   "COAXSETVERBOSE"
   "<COAXSETVERBOSE-REQUEST>"
   "<COAXSETVERBOSE-RESPONSE>"
   "COAXCONFIGUREOAMODE"
   "<COAXCONFIGUREOAMODE-REQUEST>"
   "<COAXCONFIGUREOAMODE-RESPONSE>"
   "COAXGETSENSORLIST"
   "<COAXGETSENSORLIST-REQUEST>"
   "<COAXGETSENSORLIST-RESPONSE>"
   "COAXRESET"
   "<COAXRESET-REQUEST>"
   "<COAXRESET-RESPONSE>"
   "COAXREACHNAVSTATE"
   "<COAXREACHNAVSTATE-REQUEST>"
   "<COAXREACHNAVSTATE-RESPONSE>"
  ))

