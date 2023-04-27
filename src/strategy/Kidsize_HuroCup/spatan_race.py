#!/usr/bin/env python
#coding=utf-8
import sys
import rospy
import numpy as np
import math
from Python_API import Sendmessage
from lift_and_carry import LiftandCarry
from wall_climbing import WallClimbing

send = Sendmessage()

def Strategy_select():
#策略選擇
    if send.DIOValue == 1 or send.DIOValue == 2 or send.DIOValue == 3 or  send.DIOValue == 9 or send.DIOValue == 10 or send.DIOValue == 11:
        return "Lift_and_Carry_off"
    elif send.DIOValue == 17 or send.DIOValue == 18 or  send.DIOValue == 19 or  send.DIOValue == 25 or  send.DIOValue == 26 or  send.DIOValue == 27:
        return "Lift_and_Carry_on"
    elif send.DIOValue == 4 or send.DIOValue == 12:
        return "Wall_Climb_off"
    elif send.DIOValue == 20 or send.DIOValue == 28:
        return "Wall_Climb_on"

if __name__ == '__main__':
    try:
        lc = LiftandCarry()
        cw = WallClimbing()
        rospy.init_node('SR_strategy', anonymous=True, log_level=rospy.INFO)   #初始化node
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            # start    = rospy.get_time()
            strategy = Strategy_select()
            rospy.loginfo(strategy)
            if strategy == "Lift_and_Carry_off" or strategy =="Lift_and_Carry_on":
                lc.main(strategy)
            elif strategy == "Wall_Climb_off" or strategy =="Wall_Climb_om":
                cw.main(strategy)
            # end      = rospy.get_time()
            # rospy.logdebug(f'策略計算總時間: {end-start}')
            r.sleep()
    except rospy.ROSInterruptException:
        pass

# ░░░░░░░░░▄░░░░░░░░░░░░░░▄░░░░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂▀████▀▄▄⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂▄█ 
# ░░░░░░░░▌▒█░░░░░░░░░░░▄▀▒▌░░░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂█▀░░░░▀▀▄▄▄▄▄⠂⠂⠂⠂▄▄▀▀█⠂⠂
# ░░░░░░░░▌▒▒█░░░░░░░░▄▀▒▒▒▐░░░░░░   ⠂⠂⠂▄⠂⠂⠂⠂⠂⠂⠂█░░░░░░░░░░░▀▀▀▀▄░░▄▀ ⠂⠂
# ░░░░░░░▐▄▀▒▒▀▀▀▀▄▄▄▀▒▒▒▒▒▐░░░░░░  ⠂▄▀░▀▄⠂⠂⠂⠂⠂⠂▀▄░░░░░░░░░░░░░░▀▄▀⠂⠂⠂⠂⠂
# ░░░░░▄▄▀▒░▒▒▒▒▒▒▒▒▒█▒▒▄█▒▐░░░░░░   ▄▀░░░░█⠂⠂⠂⠂⠂⠂█▀░░░▄█▀▄░░░░░░▄█⠂⠂⠂⠂⠂
# ░░░▄▀▒▒▒░░░▒▒▒░░░▒▒▒▀██▀▒▌░░░░░░   ▀▄░░░░░▀▄⠂⠂⠂█░░░░░▀██▀░░░░░██▄█ ⠂⠂⠂⠂
# ░░▐▒▒▒▄▄▒▒▒▒░░░▒▒▒▒▒▒▒▀▄▒▒▌░░░░░  ⠂⠂▀▄░░░░▄▀⠂█░░░▄██▄░░░▄░░▄░░▀▀░█ ⠂⠂⠂⠂
# ░░▌░░▌█▀▒▒▒▒▒▄▀█▄▒▒▒▒▒▒▒█▒▐░░░░░  ⠂⠂⠂█░░▄▀⠂⠂█░░░░▀██▀░░░░▀▀░▀▀░░▄▀⠂⠂⠂⠂
# ░▐░░░▒▒▒▒▒▒▒▒▌██▀▒▒░░░▒▒▒▀▄▌░░░░  ⠂⠂█░░░█⠂⠂█░░░░░░▄▄░░░░░░░░░░░▄▀⠂⠂⠂⠂⠂
# ░▌░▒▄██▄▒▒▒▒▒▒▒▒▒░░░░░░▒▒▒▒▌░░░░  ⠂█░░░█⠂⠂█▄▄░░░░░░░▀▀▄░░░░░░▄░█ ⠂⠂⠂⠂⠂
# ▀▒▀▐▄█▄█▌▄░▀▒▒░░░░░░░░░░▒▒▒▐░░░░  ⠂⠂▀▄░▄█▄█▀██▄░░▄▄░░░▄▀░░▄▀▀░░░█ ⠂⠂⠂⠂⠂
# ▐▒▒▐▀▐▀▒░▄▄▒▄▒▒▒▒▒▒░▒░▒░▒▒▒▒▌░░░  ⠂⠂⠂⠂▀███░░░░░░░░░▀▀▀░░░░▀▄░░░▄▀⠂⠂⠂⠂⠂
# ▐▒▒▒▀▀▄▄▒▒▒▄▒▒▒▒▒▒▒▒░▒░▒░▒▒▐░░░░  ⠂⠂⠂⠂⠂⠂▀▀█░░░░░░░░░▄░░░░░░▄▀█▀ ⠂⠂⠂⠂⠂
# ░▌▒▒▒▒▒▒▀▀▀▒▒▒▒▒▒░▒░▒░▒░▒▒▒▌░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂▀█░░░░░▄▄▄▀░░▄▄▀▀░▄▀ ⠂⠂⠂⠂⠂
# ░▐▒▒▒▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▒▄▒▒▐░░░░░  ⠂⠂⠂⠂⠂⠂⠂⠂⠂⠂▀▀▄▄▄▄▀⠂▀▀▀⠂▀▀▄▄▄▀⠂⠂⠂⠂⠂
# ░░▀▄▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▄▒▒▒▒▌░░░░░
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++++==--::::--==+++++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++++=-...........:-=+++++++++++++++++++++++++++++++++++++++++
# ++++++++++++++++++++++++++++++++++++++++++=:..............:=++++++++++++++++++++++++++++++++++++++++
# ++++++++++++++++++++++++++++++++++++++++++-......:--:......-++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++=......:++++:.....:++++++++++++++++++++++++++++++++++++++++
# +++++++++++++++++++++++++++++++++++++++++===----=++=-......-++++++++++++++++++++++++++++++++++++++++
# ++++++++++++++++++++++++++++++++++++++++++++++++++-:......:=++++++++++++++++++++++++++++++++++++++++
# ++++++++++++++++++++++++++++++++++++++++++++++++=:......:-++++++++++++++++++++++++++++++++++++++++++
# +++++++++*%@@%*++++++++++++++++++++++++++++++++=.......-=+++++++++++++++++++++++++++++++++++++++++++
# ++++++++*@#+**%@#++++++++++++++++++++++++++++++:.....:=+++++++++++++++++++++++++++++++++++++++++++++
# ++++++++@*+==++*#@%*+++++++++++++++++++++++++++::::::-++++++++++++++++++++++++++++++++++++++++++++++
# +++++++#%------+++#@%++++++++++++++++++++++++++=======++++++++++++++++++++++++++++++++++++++++++++++
# +++++++@*-------=+++#@#++++++++++++++++++++++++......-++++++++++++++++++++++++++++++++++++++++++++++
# +++++++@=---------+++*%@#++++++++++++++++++++++......-++++++++++++++++++++++++++++++++++++++++++++++
# ++++++*@-----------=+++*%%*++++++++++++++++++++......-++++++++++++++++++++++++++++++++++++++++++++++
# ++++++*@--------=*%%#*+++*%%*++++++++++++++++++------=+++++++++++++++++++++++++++++++**#%#++++++++++
# ++++++*@--+%%##%%*-:=#@#*++#@%++++++++++++++++++++++++++++++++++++++++++++++++++++*%@%#**@*+++++++++
# +++++++@+**+=-.  .:-.  -+*++*#@%+++++++++++++++++++++++++++++++++++++++++++++++*%@%*+++++%%+++++++++
# +++++++@%+----:.          .:+++#%#++*##########*++++++++++++++++++++++++++++*#@%#+++++===%%+++++++++
# +++++++-..                 .+++++*#####*****####%%@%##*++++++++++++++++++*#%%#*++++=-----%#+++++++++
# +++++=*%%*=.      .-=+***++++++++++++++++++++++++++**#%%%%*+++++++++++*#@%#*+++==-------=@++++++++++
# ++++#@+.      -+#%%#*+++++++++++++++++++++++++++++++++++**#%%#*++++*%@%#*+++=-----------%%++++++++++
# +++%%==-  .=#%%#*+++++++++++++++++++++++++++++++++++++++++++**#%%%%#*++++++****+=------+@+++++++++++
# +++##%=.=#@#*++++++++++++++++++++++++++++++++++++++++++++++++++*****+--=++++======----=@#+++++++++++
# +++@#-*%#*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++=:.      :--:.:--%%++++=+++++++
# +++@@%#*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++-.    .-*%%**#+=+++=+++++++
# +#@%*+++++++++++++++-:-++++++++++++++++++++++++++++++++++++++++++++++++++++=:      :+%%*+-+++-++++++
# %%*++++++++++++++=: .=++++++++++++++*%*+++++++++++++++++++++++++++++++++++++++=.   .  -%#+-++*-***++
# *++++++++++++++-.  -**++++++++++++*#@@*+++++++++++++++++++++++++++++++++++++++**=  .+#-:#++=*++=***+
# +++++++++++++:   .*#+++++++++++++#@*+@++++++++++++++++++++++++++++++++++++++++++##:  -%#+++-+++=+**+
# +++++++++++-    -%*++++++++++++#@*: +%++++++++++++++++++++++++*%*++++++++++++++++*%+  =%@++=:*=***+*
# ++++++++++:    *@*++++++++++*#@*:   +@++++++++++++++++++++++++#@%+++++++++++++++++*%#.=@%*+-+++=*+++
# +++++++++:   .#%*++++++++*#%%+.     -@*+++++++++++++++++++++++%*%%++++++++++++++++++%%.*%+*-*+++*+++
# ++++++++:   .%%+++++++*#%%*-         #%++++++++++++++++++++++*@::@#+++++++**+++++++++%%:%#*--+++++++
# +++++++:   .#%++++*##%%+-.           :@#++*++++++++++++++++++%+  -@#++++++*%#+++++++++%%*%+++++*++++
# ++++++-    #%*##%@@%%*===--..         =@#+*+++++++++++++++++#%.   -@#++++++*@#+++++++++%@*++++++++++
# +++++=    +@##@@@@@@@@@@@@@@@%*=-.     =@#*#+++++++++++++++*@-     -@#++++++#@*++++++++*%%++++++++++
# +++++.   -@-.++#%#*+======+*#%@@@@%+-.  -%%*#++**+++++++++*@=       :%%*+++++#@*++++++++*@#+++++++++
# ++++-   .%*       :=*#%%%%%%%#+==+#@@@*- .+%*=-#%@@%####*#@+         .+@#*++++%%+++++++++#@+++++++++
# ++++.   +@.    :*%%#*+=======+*#@*- :=+*=.  :.   ..=+++*##+==++****+==::*@%#*+*@#+++++++++%%++++++++
# +++-   .%*   .*@*+===============*%#:                   =@@@@@@@@@@@@@@@%#%@@%#%@+++++++++*@#+++++++
# +++:   =@:  .%%+===================*%=                  .:::::::--=+=+*#@@@@@%#*@#+++++++++#@+++++++
# +++    #%   +@+=====================+%=                   .:+#@@%%%%%%*=:.-=*%@@@%+++++++++*@#++++++
# ++=    @+   +@+======================#%:                :*@%#*+=======+*%%*:  :=#@#+++++++++#%++++++
# ++-   .@-   .%#======================*@:               =@#+==============+*%#:  .@%*++++++++*@*+++++
# ++:   =@:    :@*=====================%%.              =@#===================*@-  #%++++++++++%#+++++
# ++:   =@.     -%#+=================+%%:              .@%=====================#@. +%++++++++++%%+++++
# ++-   =%       .+%#*=============+#@+.               :@#======================@= +%++++++++++#@+++++
# ++=   +%         .-*%%##*+++**#%@#=.                  %@+=====================@= +%++++++++++#@+++++
# **+   +%             .:=++***+=-.                     -%%====================*@: +%++++++++++*@+++++
# **+:  =@.                                              .*%*+===============+#@=  +%++++++++++#@+++++
# +%*=  :@:                                                :+%%#++========+*%@*:   *%++++++++++#@+++++
# +*@*-  %+                                 .-*#+             :=*%%%%##%%@%*-.     %#++++++++++%%+++++
# ++#%*. *@#=.                            =#@#=-%#:                .:-::..        .@#++++++**+*@*+++++
# +++%@+ .@%%@#=:                         .:.   .+%#.                             =@*++++++#%+#@++++++
# %*++#@+.=@**%@@%+-.                             .:.                             #%++++++*@#*@#++++++
# #%@%##%%+*%%+*%##%@%+-:                                                        :@#++++++%@+%%+++++++
# ++*##%##%@%+:%+#%**##@@%#*=-.                                                 :*%++++++#@*#@++++++++
# ++++++*%#.:--@:.#@#**#@#:-=*#%#*+=-:.                                  .:-=+#%@@*+++++#@#*@@*+++++++
# +++++*@*.   =@.  +@%***%@=    .:-=+*#%%#**+===--::::::::::---===++**#%%%%%##**%%+++++#@#*%*%%+++++++
# ++++*@+     +#    :%%#**#@%-           .:%#=+@#****#####**#@@@%@%+%#*++++++++#%*+++*%%*+*++#@+++++++
# +++*@#      #*     .%@#***%@+            -@*#@.         -#@@%#*#@:=@#++++++++%*+++*@%++++++#@+++++++                                                                  
#                                                           ...                                                                      
#      =+=:        .-==.                              .-=++++++++++++++=+=-:                                                          
#     -=.:=++    ++=-:-+                           :=++==++***++*+++++*++++*#*-:                                                      
#     #.::-.#    #.-::.#.                       .=+==+*+++***++++++++##++++*+*%##+:                                                   
#    :+::::+-    -=::::=-                   =+:++-+*++++++*=#+++++++#=#+++*##*****##+..                                               
#    *::::.#     .#.-:-:*                   -#*+++++******-+#++****#-=#++##**#*******#**:                                             
#    #.:::==      +:-:-.#.                 .*+=+****=----+=+*#+=---=-+####*##**#**#**#=+-                                             
#   .*.:::+*+-    -=:--:+:        .%*=-.  -%=*#-=+=-=+*####*+--+*****##***#*##*****#**+*-                                             
#   :*:--.=.:=*=  .*:--:==         :*=*#*#+-=*++==+#*+====--=*#==++**+#****##**#****#**-#.                                            
#   :*:--:.---.#. .#.---:+          :#-=#++*+-=-**++**+*-=#*++**++-**##*#***#*#*#****#*-%%.                                           
#   :*:--:---:==   #.---:*           =*--==-=+**+++++++*=+*+++++*#-#++#***#**#*********=++#-:                                         
#   .#.------:*    #.---.#           +#+---+-#*+++++++++#*++++++*-**++*###*****#***#***==-==*+                                        
#    *:-----:#.    =+===++          +#+*%#**+#+-+++++++++*++++++*-==*+#***##*#*****#**++..:-:-*:                                      
#    :*:---:*:      ....         :+##++%*+++%+*-*=  =++++++++++++=  -**:##***#*******=.::..:.--*-                                     
#     -*--=*:                 .+%#+=+*##=+++*+*-*.  :+:=+++=++++++  ..#-+*#****++++-*::.::.::::++                                     
#      .:-:++++=-:.            .-=**%-#=+++*++#=*   == -++*.-+++++ ....==+*####%**+== ::... :-:=#.                                    
#         *--:::-==++=              +=*-+++*+**=#::-*+++++++++*+++     --*++++++#==--:.:...:.-:+=#                                    
#        ==:-----==-.#.             #*+=+++#+*==#:*+*+++++++++*+++    .--**+++++*=+--:::.....==+#=                                    
#        .+++=----=:+-             .#+=++++#+*-=*:++*+++++++++*++=    ..-+#++++#==+=-=:::---=-*-                                      
#     .=+++=+==--:::-*-        .    #=-+++**+=:-*-=++*++++++++*++-     .=++*++++#*====*+==---##+                                      
#     +-::--=========.#     .=#+#*. =#-+++*++:.=+=-++#++++++++*+*:.   . ====++++**#*#=+++=++#*+*%:                                    
#     +:=============-+    .#-=.--+  %-+==++*--+*+=+=**====+++#+*==----:==+ *+++++*+*###%%##*#%=-.                                    
#     *:-:--=.:====-=+.   .%= - ::%+.%-+===+*. .*-=-==#====++=*=+       ==+.*+++++++++**%#****%.                                      
#     :*+*%=-=====:-+=    .#:.: -.==+#-+===++:::-*--*=*+====+=+*=::::-: ===:*+*++*++++**%*##**%                                       
#      :+=-=========.#.-=-.%..  .:.-+*-===###****%*#++=*====+%#%#*******#=-.*+++++++++*##*#*%%-                                       
#    .++-=====--:===:==-:+:%. :::-.:*+=-+-*-**=:::*=:+=++=-=-=*+#-::--#-++: ++++*++++**%**#+%-                                        
#    #:=====-++#:+===+++-*:+=  .:. .%+*-*=* +*.   =-. .=.:=++=+**     #.*+. =+++***++*##***+%                                         
#    -*-==-++. *-=+++++=:#.:%.  .-++=#%:=#=::*:   +:        .::-*     % ++  =+*+-::+**%***++%                                         
#     .++=*:  .:+*====+++=  ++-==.**=*@==##+ :*   #       .   . -+   =+:+-  =*-.--. #%#**+++%                                         
#       ..   -*==++...      +#- .=#%#=*%-##-..-+++:  .        .  -++*+.=*.. =-.=- . **#*++++%                                         
#            +:++:#       .*= .+%#=-%-.#***.....    .         .   .....*-   . -:--  #*#+++++%                                         
#            *.++.*      .#: -##-   :%:.#*=.....  .               ....-= .  . =--: +#**+++++%                                         
#            #:++.*      +-:+%=      -%:.%+         .              . .-   . . .= .*%*+*+++++%                                 .       
#           .*-++:*      .#+%:        -%--@-       ..+=====++             .=*=#**#*%*+++++++%                                 .       
#           :+=+*:*       :@-:         =%%##*-      .--:::--=           :==::-=#**+%*+++++++%                                         
#           -=++*:*        ==::         *%#**##+-.           ..     :=+++::-::=*++*#*++++*++%                                         
#           =-++*:*         +-:.       :####*##*##%*++==----====++#+==::-----:*=++##+++++*++%                                         
#           +-*+*:*        =.*-:.      -*%%**#**#*%*#*---=+#=-:--=+**--------:*+++##+++++*++%.                                        
#           +:*+*.*        *  *::.      -#%##%*+*+%*#=+*++*****+*++*----------++++*#+*+++*++%:                                        
#           *:***.*       :=  .#::.      =#%##*+*+#*#:**:.+=+=-*:=*--=++-=-+-+-++#+*+*++*#*+#-                                        
#           *:***:*       :+   .#::.      +%##+++*#*#=*=:.-*+%-=-....  .=%#%+#-++%+*+*++#**+*+                                        
#           *---==+       -+    .*:::    .+#%%=++#*#*##. ..++: ......    :#**#-++%##**++#**++#                                        
#       =++-.-=--:  :-==. :*     .*-::. :=**-*=++%*#=*+ . =.+ . ... . . . =%%%-++#*#%+++#*#++%:                                       
#      :*-==+*.   =+===:* .#  -.   ++::. +.= ===*-*+*#-.  :=.  .-:-::- . .:%%%++=#*##++*%*#+=**                                       
#      *:***-*.   +:***=+: #  =.    :*-::.*.+=+=* :*++***+===+++++++#=.   .%%%%+++%#%==##%#+==%.                                      
#     .*-***-+    :*=***-+ +: =:      #+.::+--=++.#++++++++++++++++++**=.. %%%%%%%%%#==#***##*+*                                      
#     =-***-*.     *:#*#:# :+ --      *+=:::+==#-#*++++++++++++++++++%*+*=.#%%%%%%%%+==%****%*=#:                                     
#     *:***:+      ==***=+: +..*      #-%::--===#%+++++++++++++++++++%:=***#*%%%%%%%===%%%#**##=#                                     
#     #:#**=*+=.   .*=***-= .= *.    .#:*+.=--+*:%*+++++++*+++++++++*%- .-+*%%%%%%%%===%%%%#**#**-                                    
#    .#-#*+--=-++.  #-#*#:*    .+    .*:*::=--#*+-%*++++++#+++++++++#@=  -*=%%%#%%%@=-=%%#*****#=#                                    
#    .#-***.**#=+:  #:#*#:#     -:   :+-+ ----%:=:*%##++++*#++++***#%%+:*#####****#%=-=%%#****#%+#:                                   
#     #-#******-+   *:###-#.     .   -=*: =:-=* :*-%#%%***+*#*%##%**#%##***********#=-=#****##%%==+                                   
#     #:#*#*##:#.   +:###=*.         =-#  =--+: ==:%++####**#***++*##**************#+--%****#%%%=-#                                   
#     =-*#*##-*:    -*==++*          **-  =-++ :-.##*+**###*+++++*+*%***************#--%***#%%%@-:#.                                  
#     .#-*##-*-       ...           -#*#.:=-*+ =  *##**++++++++++*#%#***************%--#***##%%#+-*-                                  
#      :*===*-                      *+####=-#=:- .%**+++++++++*#%#*#%***************#=:+#**%%++*#-+=                                  
#        :-:                       -*=%=..+-%=*  +#++++++*####*+++*%%#%**#***********#:=*#%**++*%-==                                  
#                                  #-##:  *-#-+ .%########+*####***%%%%####**********#=-=+:-*++#%=+=                                  
#                                 +##*    *:+++ +#+++++++%#+++++***%%%##**###++%#*%%+#*:-+::#+=%#=+-                                  
#                                 .#=.    =-+++.#+++++=+*+*#+++++++###**#*=.. ..+:**++-+::*.#=+++-*.                                  