%% Script to determine specs of gear wheels
%% writen by Pascal Schoppmann, 20.12.2018, Arosa

close all, clear all, clc;

%% Input Parameters

D = 60; %Teilkreisdurchmesser
B = 6; %Zahnbreite
L = 16; %Totale Länge
H = 44; %Durchmesser Flansch
P = 4; %Durchmesser Bohrung

prompt = {'Teilkreisdurchmesser','Zahnbreite','Eingabe Länge (Total)','Durchmesser Flansch','Durchmesser Bohrung'};
dlg_title = 'Eingabe Zanhrad';
num_lines = 1;
def = {'','','','',''};
answer = inputdlg(prompt, dlg_title, num_lines, def);

D = str2num(answer(1));
B = str2num(answer(2));
L = str2num(answer(3));
H = str2num(answer(4));
P = str2num(answer(5));

result = 1;

weight = msgbox(["Gewicht Zahnrad = ', num2str(result)]);

