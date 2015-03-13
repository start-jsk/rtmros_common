if [ ! "$RTCTREE_NAMESERVERS" ]; then
  export RTCTREE_NAMESERVERS=localhost:15005
fi

# enable set alias on non-interactive shell
case "$CATKIN_SHELL" in
    "bash" )
        shopt -s expand_aliases
        ;;
esac

# these completion settings are copied from rosbash
case "$CATKIN_SHELL" in
    "bash" )
        complete -F "_roscomplete_launch" -o filenames "rtmlaunch" ;;
    "zsh" )
        compctl -/g '*.(launch|test)' -x 'p[1]' -K "_roscomplete" -tx - 'p[2]' -K _roscomplete_launchfile -- + -x 'S[--]' -k "(--files --args --nodes --find-node --child --local --screen --server_uri --run_id --wait --port --core --pid --dump-params)" -- "rtmlaunch"
esac
#echo ";; set RTCTREE_NAMESERVERS=$RTCTREE_NAMESERVERS"
