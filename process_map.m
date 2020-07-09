clc; clear all; close all;

normMap = fopen('C:\Users\39380\source\repos\Pioneer3Robot_MappingAndPthPlanning\Pioneer3Robot_MappingAndPthPlanning\NormalMap.txt', 'r');
formatSpec = '%d';
sizeMap = [61 61];
map = fscanf(normMap,formatSpec,sizeMap);

map = map';
map = imclearborder(map);

filled = imfill(map);

filled(1,:) = 1;
filled(61,:) = 1;
filled(:,1) = 1;
filled(:,61) = 1;

processedMap = fopen('C:\Users\39380\source\repos\Pioneer3Robot_MappingAndPthPlanning\Pioneer3Robot_MappingAndPthPlanning\ProcessedMap.txt', 'w');

for i=1:size(filled,1)
    fprintf(processedMap,'%d ',filled(i,:));
    fprintf(processedMap,'\n');
end
%imshow(filled)