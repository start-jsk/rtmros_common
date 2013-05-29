# aptitude install spacenavd libspnav-dev libspnav0

sudo service spacenavd start;
sleep 5 ;
rosrun spacenav_node spacenav_node ;