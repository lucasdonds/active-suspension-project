function findSpeed()
numCounts = double(10000.7); 
cpr = 3415;
duration = double (100.3);
rpm = double(abs(numCounts));
rpm = rpm/duration;
rpm = rpm/cpr;
rpm = rpm*60000