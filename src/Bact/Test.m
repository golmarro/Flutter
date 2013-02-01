%%

bactParams = BactParams('clinverse');
bact2 = Bact2(bactParams);
bact2.simStyle = 'r--';
bact2.sim(10);