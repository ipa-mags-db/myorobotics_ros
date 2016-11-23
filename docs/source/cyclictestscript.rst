::

  #!/bin/bash

  echo "First - no load"

  echo -e "1. Test  -t1 -p80 -n -i 10000 -l 10000"

  ./cyclictest -t1 -p80 -n -i 10000 -l 10000

  echo -e "expected: min 6, max 50, avg 10"

  echo -e "\n\n2. Test -t1 -p80 -i 10000 -l 10000"

  ./cyclictest -t1 -p80 -i 10000 -l 10000

  echo -e "expected: min 20, max 60, avg 31"

  echo -e "\n\n3. Test -t1 -p80 -i 500 -n -l 10000"

  ./cyclictest -t1 -p80 -i 500 -n -l 10000

  echo -e "expected: min 5, max 48, avg 7"

  echo -e "\n\n4. Test -t1 -p80 -i 500 -l 10000"

  ./cyclictest -t1 -p80 -i 500 -l 10000

  echo -e "expected: min 12, max 78, avg 16"
