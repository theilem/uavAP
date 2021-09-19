clc
regions(length(Rp0_20)) = struct('A', [], 'b', []);
for i = 1: length(Rp0_20)
    r = struct('A', reshape(Rp0_20(i).A, 1, []), 'b', Rp0_20(i).b);
    regions(i) = r;
end
reachableSet = struct('regions', regions);
reachableJson = jsonencode(reachableSet, 'PrettyPrint', true);
fid = fopen('/tmp/reachable20.json', 'w');
fprintf(fid, reachableJson);
fclose(fid);