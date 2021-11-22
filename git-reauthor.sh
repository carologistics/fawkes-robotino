#!/bin/bash

if [ $# -lt 3 ] ; then
  echo "Wrong number of arguments. Got $#, expected 3 or 4."
  echo "usage: $0 [-f] <old_email> <new_email> <first_commit>"
  exit 1
fi

if [ "$1" = "-f" ] ; then
  FORCE=-f
  OLD_EMAIL=$2
  NEW_EMAIL=$3
  COMMIT=$4
else
  FORCE=
  OLD_EMAIL=$1
  NEW_EMAIL=$2
  COMMIT=$3
fi

echo "Changing email $OLD_EMAIL to $NEW_EMAIL."
 
FILTER_BRANCH_SQUELCH_WARNING=1 git filter-branch $FORCE --env-filter "
 
an=\"\$GIT_AUTHOR_NAME\"
am=\"\$GIT_AUTHOR_EMAIL\"
cn=\"\$GIT_COMMITTER_NAME\"
cm=\"\$GIT_COMMITTER_EMAIL\"
 
if [ \"\$GIT_COMMITTER_EMAIL\" = \"$OLD_EMAIL\" ]
then
    cm=\"$NEW_EMAIL\"
fi
if [ \"\$GIT_AUTHOR_EMAIL\" = \"$OLD_EMAIL\" ]
then
    am=\"$NEW_EMAIL\"
fi
 
export GIT_AUTHOR_NAME=\"\$an\"
export GIT_AUTHOR_EMAIL=\"\$am\"
export GIT_COMMITTER_NAME=\"\$cn\"
export GIT_COMMITTER_EMAIL=\"\$cm\"
" $COMMIT...HEAD
