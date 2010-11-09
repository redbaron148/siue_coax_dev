(defpackage coax_msgs-srv
  (:use cl
        roslisp-msg-protocol)
  (:export
   "COAXSETCONTROLPARAMETERS"
   "<COAXSETCONTROLPARAMETERS-REQUEST>"
   "<COAXSETCONTROLPARAMETERS-RESPONSE>"
   "COAXSENDSTRING"
   "<COAXSENDSTRING-REQUEST>"
   "<COAXSENDSTRING-RESPONSE>"
   "COAXSETRAWCONTROL"
   "<COAXSETRAWCONTROL-REQUEST>"
   "<COAXSETRAWCONTROL-RESPONSE>"
   "COAXGETTRIMMODE"
   "<COAXGETTRIMMODE-REQUEST>"
   "<COAXGETTRIMMODE-RESPONSE>"
   "COAXSETCONTROL"
   "<COAXSETCONTROL-REQUEST>"
   "<COAXSETCONTROL-RESPONSE>"
   "COAXCONFIGURECOMM"
   "<COAXCONFIGURECOMM-REQUEST>"
   "<COAXCONFIGURECOMM-RESPONSE>"
   "COAXREQUESTSTATE"
   "<COAXREQUESTSTATE-REQUEST>"
   "<COAXREQUESTSTATE-RESPONSE>"
   "COAXREACHNAVSTATE"
   "<COAXREACHNAVSTATE-REQUEST>"
   "<COAXREACHNAVSTATE-RESPONSE>"
   "COAX3DSETCONTROLMODE"
   "<COAX3DSETCONTROLMODE-REQUEST>"
   "<COAX3DSETCONTROLMODE-RESPONSE>"
   "COAXSETTIMEOUT"
   "<COAXSETTIMEOUT-REQUEST>"
   "<COAXSETTIMEOUT-RESPONSE>"
   "COAXSETTRIMMODE"
   "<COAXSETTRIMMODE-REQUEST>"
   "<COAXSETTRIMMODE-RESPONSE>"
   "COAXRESET"
   "<COAXRESET-REQUEST>"
   "<COAXRESET-RESPONSE>"
   "COAXGETSENSORLIST"
   "<COAXGETSENSORLIST-REQUEST>"
   "<COAXGETSENSORLIST-RESPONSE>"
   "COAXSETACKMODE"
   "<COAXSETACKMODE-REQUEST>"
   "<COAXSETACKMODE-RESPONSE>"
   "COAXGETCONTROLPARAMETERS"
   "<COAXGETCONTROLPARAMETERS-REQUEST>"
   "<COAXGETCONTROLPARAMETERS-RESPONSE>"
   "COAXCONFIGURECONTROL"
   "<COAXCONFIGURECONTROL-REQUEST>"
   "<COAXCONFIGURECONTROL-RESPONSE>"
   "COAXSETVERBOSE"
   "<COAXSETVERBOSE-REQUEST>"
   "<COAXSETVERBOSE-RESPONSE>"
   "COAXCONFIGUREOAMODE"
   "<COAXCONFIGUREOAMODE-REQUEST>"
   "<COAXCONFIGUREOAMODE-RESPONSE>"
   "COAXGETVERSION"
   "<COAXGETVERSION-REQUEST>"
   "<COAXGETVERSION-RESPONSE>"
  ))

