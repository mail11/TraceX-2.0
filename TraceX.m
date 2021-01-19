%%%%%%%%%%%%%%%%%%%%%%   TraceX

% TraceX is a MatLab application for quantifying the exchange and diffusion
% properties of solid materials based on isotope tracer profiles.
% For support with this application please contact Sam Cooper at:
% camsooper@gmail.com

% Copyright (c) 2016, Samuel J Cooper
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice, this
%    list of conditions and the following disclaimer.
% 2. Redistributions in binary form must reproduce the above copyright notice,
%    this list of conditions and the following disclaimer in the documentation
%    and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
% ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

function varargout = TraceX(varargin)
% TRACEX MATLAB code for TraceX.fig v1.05
%      TRACEX, by itself, creates a new TRACEX or raises the existing
%      singleton*.
%
%      H = TRACEX returns the handle to a new TRACEX or the handle to
%      the existing singleton*.
%
%      TRACEX('CALLBACK',hObject,eventData,hand,...) calls the local
%      function named CALLBACK in TRACEX.M with the given input arguments.
%
%      TRACEX('Property','Value',...) creates a new TRACEX or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before TraceX_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to TraceX_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHAND

% Edit the above text to modify the response to help TraceX

% Last Modified by GUIDE v2.5 13-Jan-2021 19:02:20
% Begin initialization code - DO NOT EDIT
gui_Singleton = 0;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @TraceX_OpeningFcn, ...
    'gui_OutputFcn',  @TraceX_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before TraceX is made visible
function TraceX_OpeningFcn(hObject, eventdata, hand, varargin)
% get(hand.Xlimit,'Position')
hand.ImagePlotPos=[.01 .09 .71 .71];%norm
hand.ImagePlotPosXlim=[.605 .0475 .05 .03];%norm
hand.ImagePlotPosYlim=[.043 .782 .049 .030];%norm

hand.GraphPlotPos=[.08 .09 .66 .71];%norm
hand.GraphPlotPosXlim=[.708 .05 .05 .033];%norm
hand.GraphPlotPosYlim=[.028 .782 .05 .033];%norm

hand.output = hObject;
% axes(hand.MainAxes)
%imshow('Electroceramics Group.png')
[A, map, alpha] = imread('TraceX_logo.png');
h = imshow(A, map);
set(h, 'AlphaData', alpha);
% set(hand.MainAxes, 'Visible','off');
% set(hand.MinorAxes,'Visible','off')
%%%Multiline tooltips (can't be done in guide)
set(hand.Fit_Start, 'Tooltipstring', ...
    sprintf(['The distance from the surface before which the fitting algorithm will ignore. \n',...
    'If the box is green, then the value has been automatically set as the \n',...
    'pixel depth which first has 90%% of the mean counts.']));
set(hand.Fit_End, 'Tooltipstring', ...
    sprintf(['The distance from the surface after which the fitting algorithm will ignore. \n',...
    'If the box is green, then the value has been automatically set as the \n',...
    'pixel depth which first falls below the background.']));
set(hand.C_bg, 'Tooltipstring', ...
    sprintf(['Background 18O isotopic fraction \n',...
    'If the box is green, then the value has been automatically \n',...
    'set as the modal value of the profile']));
set(hand.AlignAngle, 'Tooltipstring', ...
    sprintf(['Select an alignment angle for the sample\n',...
    'If the box is green, then the value has been automatically \n',...
    'set using the method specified above']));
set(hand.ROI, 'Tooltipstring', ...
    sprintf(['Specify a region of interest using the mouse\n',...
    'to place vertices of polygone, then double-click \n',...
    'on first vertex to complete shape']));
set(hand.GenerateProfiles, 'Tooltipstring', ...
    sprintf(['Generate a range of profiles to illustrate the effect of masking and alignment.\n',...
    'Profile used for fitting is specified using dropdown box above.']));
hand.reset=hand;
guidata(hObject, hand);
% --- Outputs from this function are returned to the command line.
function varargout = TraceX_OutputFcn(hObject, eventdata, hand)
% set(gcf,'units','centimeters','outerposition',[0 3 25 20]);
varargout{1} = hand.output;
% set(gcf, 'Units' , 'Normalized');



%% --- Executes on button press in LoadProfileData.
function [hand]=LoadProfileData_Callback(hObject, eventdata, hand)
hand.ProLen_Or=500;
set(hand.Fit_End,'String',250);
[hand.FileName, hand.PathName] = uigetfile('*.*');%(FILTERSPEC, TITLE);
if hand.FileName==0
    return
end
if ispc
    hand.ProfileData_Or=textread([hand.PathName,'\',hand.FileName])';%,'%n'
else
    hand.ProfileData_Or=textread([hand.PathName,hand.FileName])';%,'%n'
end
[a, b]=size(hand.ProfileData_Or);
if min(a,b)>1
    hand.XData_Or=hand.ProfileData_Or(1,:);
    hand.ProfileData_Or=hand.ProfileData_Or(2,:);
end
hand.ProfileData_Fund=hand.ProfileData_Or;
set(hand.Xlimit,'String',get(hand.ProfileLength,'String'));
set(hand.CprimeY,'State','off')
set(hand.GBplot,'State','off')
set(hand.SurfPos,'String',0)
if ispc
    hand.SaveName_Or=[hand.PathName,'\',hand.FileName(1:end-4),...
        '_Norm18Prof.txt'];
else
    hand.SaveName_Or=[hand.PathName,hand.FileName(1:end-4),...
        '_Norm18Prof.txt'];
end

set(hand.ProfDataSaveName, 'String',hand.FileName);
set(hand.PixelNo,'String',length(hand.ProfileData_Or));
set(hand.PixelNo,'BackgroundColor',[0,1,0]);
set(hand.Fit_End,'String',get(hand.ProfileLength,'String'));
Fit_End_Callback(hObject, eventdata, hand)
set(hand.Fit_End,'BackgroundColor',[0,1,0]);
[hand]=DataPlotOnly(hObject, eventdata, hand);
guidata(hObject, hand);


function ProfileLength_Callback(hObject, eventdata, hand)
% set(hand.BoundCondMode, 'Value',1)
if isfield(hand,'ProLen_Or')
    LenRat=str2double(get(hand.ProfileLength,'String'))/hand.ProLen_Or;
    hand.ProLen_Or=str2double(get(hand.ProfileLength,'String'));
    set(hand.SurfPos,'String',roundsf(LenRat*str2double(get(hand.SurfPos,'String')),3,'round'));
    set(hand.Fit_Start,'String',roundsf(LenRat*str2double(get(hand.Fit_Start,'String')),3,'round'));
    set(hand.Fit_End,'String',roundsf(LenRat*str2double(get(hand.Fit_End,'String')),3,'round'));
end
set(hand.Xlimit,'String',get(hand.ProfileLength,'String'));
Xlimit_Callback(hObject, eventdata, hand);
set(hand.ProfileLength,'BackgroundColor',[1,1,1]);
hand.ProLen_num=str2double(get(hand.ProfileLength,'String'));
if isfield(hand,'PlotType')
    if hand.PlotType==2
        [hand] =PlotButton_Callback(hObject, eventdata, hand);
    elseif hand.PlotType==3
        [hand]=DataPlotOnly(hObject, eventdata, hand);
    end
end
[hand]=Approx_Dandk(hObject,hand);
guidata(hObject, hand);

function ProfileLength_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function PixelNo_Callback(hObject, eventdata, hand)
set(hand.PixelNo,'BackgroundColor',[1,1,1]);
hand.PixelNo_num=str2double(get(hand.PixelNo, 'String'));
if isfield(hand,'PlotType')
    if hand.PlotType==2
        [hand] =PlotButton_Callback(hObject, eventdata, hand);
    end
end
guidata(hObject, hand);
function PixelNo_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function SurfPos_Callback(hObject, eventdata, hand)
set(hand.SurfPos,'BackgroundColor',[1,1,1]);
SurfPos=str2double(get(hand.SurfPos,'String'));
if SurfPos<0
    set(hand.SurfPos,'String',0)
elseif SurfPos>str2double(get(hand.ProfileLength,'String'))
    set(hand.SurfPos,'String',str2double(get(hand.ProfileLength,'String'))/2)
end

switch hand.CurrentPlot
    case 'Image'
    case 'Align'
        delete(hand.SurfLine2);
        SurfPos=str2double(get(hand.SurfPos,'String'));
        hand.SurfLine2=line([SurfPos, SurfPos],[-5000 5000],...
            'LineWidth',2,'Color',[0 1 1],'LineStyle',':');
    case 'Mask'
        delete(hand.SurfLine2);
        SurfPos=str2double(get(hand.SurfPos,'String'));
        hand.SurfLine2=line([SurfPos, SurfPos],[-5000 5000],...
            'LineWidth',2,'Color',[0 1 1],'LineStyle',':');
    case 'Generate'
        delete(hand.SurfLine2);
        SurfPos=str2double(get(hand.SurfPos,'String'));
        hand.SurfLine2=line([SurfPos, SurfPos],[-5000 5000],...
            'LineWidth',2,'Color',[0 1 1],'LineStyle',':');
    case 'PlotButton'
        [hand] =PlotButton_Callback(hObject, eventdata, hand);
    case 'DataPlotOnly'
        [hand]=DataPlotOnly(hObject, eventdata, hand);
    case 'Fit'
        [hand] =PlotButton_Callback(hObject, eventdata, hand);
        %         FitButton_Callback(hObject, eventdata, hand)
end
guidata(hObject, hand);

function SurfPos_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function [hand]=DataPlotOnly(hObject, eventdata, hand)
hand.CurrentPlot='DataPlotOnly';
hand.PlotType=3;

set(hand.Ylimit,'Position',hand.GraphPlotPosYlim);
set(hand.Xlimit,'Position',hand.GraphPlotPosXlim);
set(hand.Ylimit,'Visible','on');
set(hand.Xlimit,'Visible','on');

[hand]=BuildX(hObject,hand);

plot(hand.Xdata*1e6,hand.ProfileData);
Xlimit_Callback(hObject, eventdata, hand);
set(hand.Ylimit,'String',roundsf(max(hand.ProfileData_Or)*1.1,2,'ceil'));
if ~isfield(hand,'PlotFlag')
    Ylimit_Callback(hObject, eventdata, hand);
end
legend('Data');xlabel('Depth /\mum');ylabel('Isotopic Fraction');
set(gca,'position',hand.GraphPlotPos);
guidata(hObject, hand);

function C_bg_Callback(hObject, eventdata, hand)
%PlotButton_Callback(hObject, eventdata, hand);
if hand.PlotType==1
    if isfield(hand,'C_bgLine')
        delete(hand.C_bgLine);
    end
    C_bg=str2double(get(hand.C_bg,'String'));
    hand.C_bgLine=line([-5000 5000],[C_bg, C_bg],...
        'LineWidth',2,'Color',[0 1 0],'LineStyle',':');
end
set(hand.C_bg,'BackgroundColor',[1,1,1]);
PlotProperties(hObject, eventdata, hand)
guidata(hObject, hand);
function C_bg_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function C_gas_Callback(hObject, eventdata, hand)
%PlotButton_Callback(hObject, eventdata, hand);
guidata(hObject, hand);
function C_gas_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function D1_Callback(hObject, eventdata, hand)
guidata(hObject, hand);
function D1_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function D2_Callback(hObject, eventdata, hand)
guidata(hObject, hand);
function D2_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function k1_Callback(hObject, eventdata, hand)
guidata(hObject, hand);
function k1_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function k2_Callback(hObject, eventdata, hand)
guidata(hObject, hand);
function k2_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function t1_Callback(hObject, eventdata, hand)
%PlotButton_Callback(hObject, eventdata, hand);
% [hand]=Approx_Dandk(hObject,hand);
guidata(hObject, hand);
function t1_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function t2_Callback(hObject, eventdata, hand)
%PlotButton_Callback(hObject, eventdata, hand);
% [hand]=Approx_Dandk(hObject,hand);
set(hand.t2,'String',abs(str2double(get(hand.t2,'String'))));
if str2num(get(hand.t2,'String')) == 0
    set(hand.k2FitCheck,'Value',0);
end
guidata(hObject, hand);
function t2_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Xlimit_Callback(hObject, eventdata, hand)
set(hand.Xlimit,'Visible','on');
%
if length(hand.CurrentPlot)==10
    %     if length(get(hand.XprimeTog,'State'))==2
    %         hand.Xlimit_num=3;
    %     else
    hand.Xlimit_num=str2double(get(hand.Xlimit, 'String'));
    %     end
else
    hand.Xlimit_num=str2double(get(hand.Xlimit, 'String'));
end
xlim([0 hand.Xlimit_num]);
set(gca,'XTick',linspace(0,hand.Xlimit_num,11))
guidata(hObject, hand);
function Xlimit_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Ylimit_Callback(hObject, eventdata, hand)
set(hand.Ylimit,'Visible','on');
hand.Ylimit_max=str2double(get(hand.Ylimit, 'String'));
if length(get(hand.GBplot,'State'))==2;
    ylim_min=hand.Ylimit_min;
else
    if get(hand.ErrorCheck, 'Value')==1;
        %     set(hand.ErrorCheck, 'Value',0); %%%possibly wrong
        ylim_min=hand.Ylimit_min;
    else
        ylim_min=0;
        %         set(gca,'YTick',linspace(0,hand.Ylimit_max,11))
    end
end
if ylim_min==0 && hand.Ylimit_max==0
    hand.Ylimit_max=1;
end
if ~isfinite(hand.Ylimit_max)
    hand.Ylimit_max=1;
end
ylim([ylim_min hand.Ylimit_max]);

guidata(hObject, hand);
function Ylimit_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ErrorCheck_Callback(hObject, eventdata, hand)
[hand] =PlotButton_Callback(hObject, eventdata, hand);
guidata(hObject, hand);


%% --- Executes on button press in PlotButton.
function [hand]=PlotButton_Callback(hObject, eventdata, hand);%fun
hand.CurrentPlot='PlotButton';
hand.PlotType=2;
hand.PlotFlag=1;
if isfield(hand, 'colo')
    delete(hand.colo);
end

set(hand.Ylimit,'Position',hand.GraphPlotPosYlim);
set(hand.Xlimit,'Position',hand.GraphPlotPosXlim);
set(hand.Ylimit,'Visible','on');
set(hand.Xlimit,'Visible','on');

C_bg=str2double(get(hand.C_bg, 'String'));
C_gas=str2double(get(hand.C_gas, 'String'));
D1=str2double(get(hand.D1, 'String'));
D2=str2double(get(hand.D2, 'String'));
k1=str2double(get(hand.k1, 'String'));
k2=str2double(get(hand.k2, 'String'));
t1=3600*str2double(get(hand.t1, 'String'));
t2=3600*str2double(get(hand.t2, 'String'));
ProLen_m=1e-6*str2double(get(hand.ProfileLength, 'String'));
BC=get(hand.BoundCondMode,'Value');
switch BC
    case 1
        MP_m=0;
    case 2
        MP_m=str2double(get(hand.MirrorPlane,'String'))*1e-6;
    case 3
        MP_m=str2double(get(hand.MirrorPlane,'String'))*1e-6;
end
PixelNo=str2double(get(hand.PixelNo, 'String'));

dx=ProLen_m/(PixelNo-1); %spatial step
X=0:dx:ProLen_m; %domain


[val_mirror hand.MP_idx] = min(abs(X-MP_m));
MP=hand.MP_idx;

%% Generate profiles
switch get(hand.ProfileMode,'Value')
    case 1 %Crank
        switch get(hand.BoundCondMode,'Value')
            case 1
                [X,pro]=Crank_SI_inline(...
                    C_gas,C_bg,D1,k1,t1,ProLen_m,PixelNo);
            case 2
                [X,pro]=Crank_PS_inline(...
                    C_gas,C_bg,D1,k1,t1,ProLen_m,PixelNo,BC,MP_m);
            case 3
                [X,pro]=Crank_PS_inline(...
                    C_gas,C_bg,D1,k1,t1,ProLen_m,PixelNo,BC,MP_m);
        end
        ProTypeFlag=1;
    case 2 %Back-Crank analytical
        [X,pro]=BackX_SI_inline(...
            C_gas,C_bg,D1,k1,t1,t2,ProLen_m,PixelNo);
        ProTypeFlag=1;
    case 3 %Back-Crank numerical
        [X,pro]=CN_inline(...
            C_gas,C_bg,D1,D2,k1,k2,t1,t2,ProLen_m,PixelNo,BC,MP_m);
        ProTypeFlag=3;
    case 4 %Interfacial
        L_int_m=str2double(get(hand.InterfaceDepth,'String'))*1e-6;
        r_int=k2;
        [X,pro]=CN_Interface_inline(...
            C_gas,C_bg,D1,D2,k1,r_int,t1,L_int_m,ProLen_m,PixelNo,BC,MP_m);
        ProTypeFlag=5;
    case 5 %LeClaire
end

hand.pro=pro;

hand.X=X;
Fit_Start=str2double(get(hand.Fit_Start, 'String'));
tmp = abs(X-Fit_Start*1e-6);
[val_start hand.Fit_Start_idx] = min(tmp);
Fit_End=str2double(get(hand.Fit_End, 'String'));
tmp = abs(X-Fit_End*1e-6);
[val_end hand.Fit_End_idx] = min(tmp);

%%% Legend
switch ProTypeFlag
    case 1
        ProStr='Analytical Solution';
    case 2
        ProStr='Back-Crank';
    case 3
        ProStr='Crank-Nicolson FD Simulation';
    case 4
        ProStr='Plane Sheet';
    case 5
        ProStr='Interface';
end
% if t2==0
%     ProStr='Crank';
% else
%     if D1==D2 && k1==k2
%         ProStr='Back-Crank';
%     else
%         ProStr='Crank-Nicolson FD Simulation';
%     end
% end

%%%plots
if ~isfield(hand,'ProfileData') %% Just profile
    if strcmp(get(hand.CprimeY,'State'),'on')
        pro=(pro-C_bg)/(C_gas-C_bg);
        if length(get(hand.GBplot,'State'))==2
            hand.Ylimit_max=0;
            pro=log(abs(pro));
            hand.Ylimit_min=-20;
            %             hand.Ylimit_min=roundsf(min(pro(isreal(pro))),1,'floor');
            X=((X./ ((D1*t1)^0.5) ).^(6/5))/1e6;
        else
            hand.Ylimit_max=ceil(20*max(pro))/20;
            hand.Ylimit_min=0;
        end
    else
        hand.Ylimit_max=ceil(20*max(pro))/20;
        hand.Ylimit_min=0;
    end
    if strcmp(get(hand.XprimeTog,'State'),'off')
        plot(X*1e6,pro);
    else
        X=roundsf(X/(2*sqrt(D1*(t1+t2) )),3,'round');
        plot(X,pro);
    end
    %     set(hand.Xlimit,'String',roundsf(max(X*1e6),2,'ceil'));
    
    axis normal;
    legend(ProStr);
else  %%%%%% Profile + Data
    [hand]=DataPlotOnly(hObject, eventdata, hand);
    Xdata=hand.Xdata;
    %%% Cprime or IF
    if strcmp(get(hand.CprimeY,'State'),'on') %if is on
        pro=(pro-C_bg)/(C_gas-C_bg);
        data=(hand.ProfileData-C_bg)/(C_gas-C_bg);
    else
        data=hand.ProfileData;
    end
    %Grain boundary
    
    if strcmp(get(hand.GBplot,'State'),'on')
        
        pro=log(pro);
        data=log(abs(data));
        X=((X/((D1*(t1+t2) )^0.5)).^(6/5))/1e6;
        Xdata=((Xdata/((D1*(t1+t2) )^0.5)).^(6/5))/1e6;
        hand.Ylimit_max=ceil(20*max(pro))/20;
    end
    if strcmp(get(hand.XprimeTog,'State'),'on')
        X=X/(2*sqrt(D1*(t1+t2) ))/1e6;
        Xdata=Xdata/(2*sqrt(D1*(t1+t2) ))/1e6;
        %         set(hand.Xlimit,'String',roundsf(1e6*X(find(1-(0<data<1),1,'last')),2,'ceil')); %%%%changed
    end
    prof_err=(pro(1:length(data))-data);
    plot(Xdata*1e6,data,'o',X*1e6,pro,'-r','linewidth',1,'MarkerSize',3);
    %%% Residuals
    if get(hand.ErrorCheck, 'Value')==1;
        if strcmp(get(hand.GBplot,'State'),'on')
            hand.Ylimit_max=ceil(20*max(max(prof_err),-min(min(pro),min(data))/10))/20;
            hand.Ylimit_min=-20;
            %             hand.Ylimit_min=roundsf(min(min(pro),min(data)),1,'floor');
        else
            hand.Ylimit_max=ceil(20*max(max(pro),max(data)))/20;
            hand.Ylimit_min=roundsf(min(min(prof_err),hand.Ylimit_max/-10),1,'floor'); %make it minimum .1*max for equal step
        end
        hold on
        plot(Xdata*1e6,prof_err,'g','linewidth',1);
        legend('Data',ProStr,'Residual'); %add residual
        plot([0 1e4],[0 0],'-k'); %add x-axis
        hold off
    else
        if strcmp(get(hand.GBplot,'State'),'on')
            hand.Ylimit_max=0;
            %             hand.Ylimit_min=roundsf(min(min(pro),min(data)),1,'floor');
            hand.Ylimit_min=-20;
        else
            hand.Ylimit_max=ceil(20*max(max(pro),max(data)))/20;
            hand.Ylimit_min=0;
        end
        legend('Data',ProStr);
    end
    %
    if hand.Fit_Start_idx < find(isfinite(data),1,'first')
        hand.Fit_Start_idx = find(isfinite(data),1,'first');
        set(hand.Fit_Start,'String',...
            roundsf(1e6*X(hand.Fit_Start_idx),3,'round'));
    end
    
    if hand.Fit_End_idx > find(isfinite(data),1,'last')
        hand.Fit_End_idx = find(isfinite(data),1,'last');
        set(hand.Fit_End,'String',...
            roundsf(1e6*X(hand.Fit_End_idx),3,'round'));
    end
    
    [r2 rmse] = rsquare(hand.ProfileData(hand.Fit_Start_idx:hand.Fit_End_idx),...
        hand.pro(hand.Fit_Start_idx:hand.Fit_End_idx));
    hand.r2=r2;
    hand.rmse=rmse;
    set(hand.Rsq,'String',num2str(roundsf(r2,4,'round')));
    
    [r2All rmseAll] = rsquare(hand.ProfileData(1:end),...
        hand.pro(1:length(hand.ProfileData)));
    set(hand.RsqAll,'String',num2str(roundsf(r2All,4,'round')));
    %     hand.Ylimit_max=ceil(20*max(max(pro),max(data)))/20;
end
set(hand.Ylimit,'String',hand.Ylimit_max);
Ylimit_Callback(hObject, eventdata, hand);

grid (gca,get(hand.GridLines,'State'))
set(gca,'Xcolor',[0.2 0.2 0.2]); set(gca,'Ycolor',[0.2 0.2 0.2]);
if strcmp(get(hand.GBplot,'State'),'on')
    xlim([0 10])
elseif strcmp(get(hand.XprimeTog,'State'),'on')
    xlim([0 3])
    xlabel('Normalised depth, x''');
    set(hand.Xlimit,'Visible','off');
else
    xlabel('Depth, x /\mum');
    Xlimit_Callback(hObject, eventdata, hand);
end

if strcmp(get(hand.CprimeY,'State'),'on')
    if strcmp(get(hand.GBplot,'State'),'on')
        ylabel('ln|C''|');
        xlabel('\eta^{6/5}');
        set(hand.Xlimit,'Visible','off')
        set(hand.Ylimit,'Visible','off')
    else
        ylabel('Normalised _{}^{18}O Fraction, C''');
    end
else
    ylabel('Isotopic Fraction');
end
%hand.PlotButtonFlag=1;

% set(gca,'position',[68 62 570 465]); %pixels
set(gca,'position',hand.GraphPlotPos); %norm
% set(gca,'yscale','log');
Fit_End_Callback(hObject, eventdata, hand);%
hand.CurrentPlot='PlotButton';
PlotProperties(hObject, eventdata, hand);
guidata(hObject, hand);

function Rsq_CreateFcn(hObject, eventdata, hand)
guidata(hObject, hand);

function MainAxes_CreateFcn(hObject, eventdata, hand)

function FitButton_Callback(hObject, eventdata, hand)
hand.CurrentPlot='Fit';
if ~isfield(hand,'ProfileData_Or')
    [hand]=LoadProfileData_Callback(hObject, eventdata, hand);
    if hand.FileName==0
        return
    end
end
% Change appearance of Fit_Button

FitCheck(1)=get(hand.D1FitCheck, 'Value');
FitCheck(2)=get(hand.D2FitCheck, 'Value');
FitCheck(3)=get(hand.k1FitCheck, 'Value');
FitCheck(4)=get(hand.k2FitCheck, 'Value');

C_bg=str2double(get(hand.C_bg, 'String'));
C_gas=str2double(get(hand.C_gas, 'String'));
D1=str2double(get(hand.D1, 'String'));
D2=str2double(get(hand.D2, 'String'));
k1=str2double(get(hand.k1, 'String'));
k2=str2double(get(hand.k2, 'String'));
t1=3600*str2double(get(hand.t1, 'String'));
t2=3600*str2double(get(hand.t2, 'String'));
ProfileLength=str2double(get(hand.ProfileLength, 'String'));
ProLen_m=ProfileLength*1e-6;
PixelNo=str2double(get(hand.PixelNo, 'String'));
BC=get(hand.BoundCondMode,'Value');
MP_m=str2double(get(hand.MirrorPlane,'String'))*1e-6;
[hand] =PlotButton_Callback(hObject, eventdata, hand);

Fit_Start=str2double(get(hand.Fit_Start, 'String'));
[val_start hand.Fit_Start_idx] = min(abs(hand.Xdata-Fit_Start*1e-6));
Fit_Start_idx=hand.Fit_Start_idx;


if get(hand.BoundCondMode,'Value')==2
    if str2double(get(hand.Fit_End, 'String'))>MP_m*1e6
        set(hand.Fit_End, 'String',get(hand.MirrorPlane,'String')) 
    end
end

Fit_End=str2double(get(hand.Fit_End, 'String'));
[val_end hand.Fit_End_idx] = min(abs(hand.Xdata-Fit_End*1e-6));
Fit_End_idx=hand.Fit_End_idx;
MP=hand.MP_idx;

if ~isfield(hand,'ProfileData')
    [hand]=Data_Callback(hObject, eventdata, hand);
end

DataStep=ProLen_m/(PixelNo-1);
hand.Xdata_Or=0:DataStep:ProLen_m;

SurfPos=str2double(get(hand.SurfPos, 'String'));
[val_start hand.SurfPos_idx] = min(abs(hand.Xdata_Or-SurfPos*1e-6));
hand.Xdata=hand.Xdata_Or(1:end-hand.SurfPos_idx+1);
hand.ProfileData=hand.ProfileData_Or(hand.SurfPos_idx:end);

set(hand.FitButton,'ForegroundColor',[0.5,0.5,0.5]);
set(hand.FitButton,'String','Fitting...');drawnow 

switch get(hand.ProfileMode,'Value')
    case 1 % Crank
        switch get(hand.BoundCondMode,'Value')
            case 1
                [D1,k1]=AutoFit_Crank(hand);
            case 2
                [D1,k1]=AutoFit_PlaneSheet(hand);
            case 3
                [D1,k1]=AutoFit_PlaneSheet(hand);
        end
    case 2 % BX analytical
        [D1,k1]= AutoFit_BackCrank(...
            C_gas,C_bg,D1,k1,t1,t2,hand.ProfileData,...
            hand.Xdata,Fit_Start_idx,Fit_End_idx);
        D2=D1;
        k2=k1;
    case 3 % BX Numerical
        % Start by using the analytical solution to get a good/quick ball park...
        [D1,k1]= AutoFit_BackCrank(...
            C_gas,C_bg,D1,k1,t1,t2,hand.ProfileData,...
            hand.Xdata,Fit_Start_idx,Fit_End_idx);
        D2=D1;
        k2=k1;
        tic;
        r2=0;
        tim=0;
        i=1;
        r2check(1)=str2double(get(hand.Rsq,'String'));
        %
        while r2<0.999 && tim<20
            
            [D1,k1,D2,k2]= AutoFit_BackDiffs(...
                C_gas,C_bg,D1,k1,D2,k2,t1,t2,hand.ProfileData,...
                Fit_Start_idx,Fit_End_idx,ProLen_m,...
                PixelNo,FitCheck,BC,MP_m);
            [X,pro]=CN_inline(...
                C_gas,C_bg,D1,D2,k1,k2,t1,t2,ProLen_m,PixelNo,BC,MP_m); %ImageLength
            [r2 rmse] = rsquare(hand.ProfileData(Fit_Start_idx:Fit_End_idx),...
                pro(Fit_Start_idx:Fit_End_idx));
            
            tim=toc+tim;
            i=i+1;
            r2check(i)=r2;
            if r2check(i)==r2check(i-1);
                tim=20;
            end
            
        end
        disp(['Total time for fit ',num2str(round(toc)),' seconds.'])
    case 4 % Interfacial
    case 5 % 
        [D1,k1,A_gb,Z_gb]=AutoFit_Crank_LC(hand);
end

% if t2==0
%     if PS==0 %Plane
%         [D1,k1]=AutoFit_Crank(hand);
%         if get(hand.Check_LeClaire,'Value')==1
%             [D1,k1,A_gb,Z_gb]=AutoFit_Crank_LC(hand);
%         end
%     else
%         [D1,k1]=AutoFit_PlaneSheet(hand);
%     end
% else
%     if FitCheck(2)==0 && FitCheck(4)==0
%         [D1,k1]= AutoFit_BackCrank(C_gas,C_bg,D1,k1,t1,t2,hand.ProfileData,...
%             hand.Xdata,Fit_Start_idx,Fit_End_idx);
%         D2=D1;
%         k2=k1;
%             elseif FitCheck(2)==0 && FitCheck(4)==1
%     else
%         Start by using the analytical solution to get a good/quick ball park...
%         [D1,k1]= AutoFit_BackCrank(C_gas,C_bg,D1,k1,t1,t2,hand.ProfileData,...
%             hand.Xdata,Fit_Start_idx,Fit_End_idx);
%         D2=D1;
%         k2=k1;
%         tic;
%         r2=0;
%         tim=0;
%         i=1;
%         r2check(1)=str2double(get(hand.Rsq,'String'));
%         
%         while r2<0.999 && tim<20;
%             
%             [D1,k1,D2,k2]= AutoFit_BackDiffs(C_gas,C_bg,D1,k1,D2,k2,t1,t2,hand.ProfileData,...
%                 hand.Xdata,Fit_Start_idx,Fit_End_idx,ProLen_m,PixelNo,hand,FitCheck);
%             [X,pro]=CN_inline(...
%                 C_gas,C_bg,D1,D2,k1,k2,t1,t2,ProLen_m,PixelNo,BC,MP_m); %ImageLength
%             [r2 rmse] = rsquare(hand.ProfileData(Fit_Start_idx:Fit_End_idx),...
%                 pro(Fit_Start_idx:Fit_End_idx));
%             
%             tim=toc+tim;
%             i=i+1;
%             r2check(i)=r2;
%             if r2check(i)==r2check(i-1);
%                 tim=20;
%             end
%             
%         end
%         disp(['Total time for fit ',num2str(round(toc)),' seconds.'])
%                 set(hand.FitButton,'Enable','on');
%     end
% end
D1=roundsf(D1,3,'round');
D2=roundsf(D2,3,'round');
k1=roundsf(k1,3,'round');
k2=roundsf(k2,3,'round');
%%% Set all the newly fitted values
set(hand.D1,'String',num2str(D1));
set(hand.D2,'String',num2str(D2));
set(hand.k1,'String',num2str(k1));
set(hand.k2,'String',num2str(k2));

FitPlot(hObject, eventdata, hand);

if isfield(hand,'O18pathname')
    hand.SaveName=[hand.PathName,hand.FileName(1:end-4)];
else
    hand.SaveName=[hand.PathName,hand.FileName(1:end-4)];
end
set(hand.FitButton,'ForegroundColor',[0,0,0]);
set(hand.FitButton,'String','Fit');
hand.CurrentPlot='Fit';
guidata(hObject, hand);

function FitPlot(hObject, eventdata, hand)
hand.CurrentPlot='Fit';
[hand] =PlotButton_Callback(hObject, eventdata, hand);
D1=str2double(get(hand.D1,'string'));
t1=3600*str2double(get(hand.t1,'string'));
t2=3600*str2double(get(hand.t2,'string'));
t_tot=(t1+t2) ;
% set(hand.RMSres,'String',num2str(hand.rmse));
hold on;
Fit_Start=str2double(get(hand.Fit_Start,'String'));
Fit_End=str2double(get(hand.Fit_End,'String'));
if strcmp(get(hand.GBplot,'State'),'off') && strcmp(get(hand.XprimeTog,'State'),'off')
    line([Fit_Start Fit_Start],[-40 1],'Color','k','LineStyle','--');
    line([Fit_End Fit_End],[-40 1],'Color','k','LineStyle','--');
elseif strcmp(get(hand.GBplot,'State'),'on')
    line(([Fit_Start Fit_Start]*1e-6/ ((D1*t_tot)^0.5) ).^(6/5),[-40 1],'Color','k','LineStyle','--');
    line(([Fit_End Fit_End]*1e-6/ ((D1*t_tot)^0.5) ).^(6/5),[-40 1],'Color','k','LineStyle','--');
elseif strcmp(get(hand.XprimeTog,'State'),'on')
    line([Fit_Start Fit_Start]*1e-6/(2*sqrt(D1*t_tot)),[-40 1],'Color','k','LineStyle','--');
    line([Fit_End Fit_End]*1e-6/(2*sqrt(D1*t_tot)),[-40 1],'Color','k','LineStyle','--');
end
legstr=get(legend,'string');
legstr(3:4)=[{'Fit start'},{'Fit end'}];
set(legend,'string',legstr);
hold off;
PlotProperties(hObject, eventdata, hand)
guidata(hObject, hand);

function SaveFit_Callback(hObject, eventdata, hand)
[hand] =PlotButton_Callback(hObject, eventdata, hand);
SaveName=get(hand.ProfDataSaveName,'String');
% [hand.FileName, hand.PathName,~] = ...
%         uigetfile([hand.PathName,'*.txt'],'Oxygen 18 file');%(FILTERSPEC, TITLE);
if isfield(hand,'PathName')
    [FileName,PathName]=uiputfile([hand.PathName,SaveName(1:end-4),'_Profiles.txt']);
else
    [FileName,PathName]=uiputfile('Profiles.txt');
end

fid=fopen([PathName,FileName],'wt');

ProData=zeros(size(hand.X));
if isfield(hand,'ProfileData')
    ProData(1:length(hand.ProfileData))=hand.ProfileData;
end
if isfield(hand,'O18pathname')
    hand.SaveName=[hand.PathName,hand.FileName(1:end-4)];
else
    hand.SaveName=[PathName,FileName(1:end-4)];
end
fprintf(fid,'FileName = ;%s;\n',hand.SaveName);
fprintf(fid,'Image_Length = ;%s; [um]\n',get(hand.ProfileLength,'String'));
fprintf(fid,'Pixel_Number = ;%s; [pixels]\n',get(hand.PixelNo,'String'));
fprintf(fid,'Surface_Position = ;%s; [um]\n',get(hand.SurfPos,'String'));
fprintf(fid,'Align_Angle = ;%s; [deg]\n',get(hand.AlignAngle,'String'));
fprintf(fid,'Mask_Thresh = ;%s; []\n',get(hand.MaskThresh,'String'));

D1=str2double(get(hand.D1,'String'));
D2=str2double(get(hand.D2,'String'));
k1=str2double(get(hand.k1,'String'));
k2=str2double(get(hand.k2,'String'));
t1=3600*str2double(get(hand.t1,'String'));
t2=3600*str2double(get(hand.t2,'String'));
C_bg=str2double(get(hand.C_bg,'String'));
C_gas=str2double(get(hand.C_gas,'String'));

fprintf(fid,'D1 = ;%g; [m2/s]\n',D1);
fprintf(fid,'D2 = ;%g; [m2/s]\n',D2);
fprintf(fid,'k1 = ;%s; [m/s]\n',k1);
fprintf(fid,'k2 = ;%g; [m/s]\n',k2);
fprintf(fid,'t1 = ;%g; [hours]\n',t1);
fprintf(fid,'t2 = ;%g; [hours]\n',t2);
fprintf(fid,'C_bg = ;%g; []\n',C_bg);
fprintf(fid,'C_gas = ;%g; []\n',C_gas);
fprintf(fid,'R_Squared = ;%s; []\n',get(hand.RsqAll,'String'));
fprintf(fid,'R_Squared ROI= ;%s; []\n',get(hand.Rsq,'String'));
fprintf(fid,'Fit_Start = ;%s; [um]\n',get(hand.Fit_Start,'String'));
fprintf(fid,'Fit_End = ;%s; [um]\n',get(hand.Fit_End,'String'));

if isfield(hand,'O18pathname')
    last = find(1-(0<ProData<1),1,'last');
else
    last=length(hand.X);
end
Pros(:,1)=hand.X(1:last);
Pros(:,2)=ProData(1:last);
Pros(:,3)=hand.pro(1:last);
Pros(:,4)=ProData(1:last)-hand.pro(1:last);

Pros(:,5)=hand.X(1:last)./(4*D1*(t1+t2) )^0.5;
Pros(:,6)=(ProData(1:last)-C_bg)/(C_gas-C_bg);
Pros(:,7)=(hand.pro(1:last)-C_bg)/(C_gas-C_bg);
Pros(:,8)=Pros(:,6)-Pros(:,7);
fprintf(fid,['Depth,x /m;Isotopic Fraction Data;Fitted Profile;Residual;;',...
    'Normalised Depth, x''=x/(4 D1 t)^1/2);Normalised Data;Normalised Fit;Residual\n']);
fprintf(fid,'%d;%d;%d;%d;;%d;%d;%d;%d\n',Pros');
fclose(fid);
% [FileName,PathName]=uiputfile([hand.PathName,SaveName(1:end-4),'_Fit.txt']);
% fid=fopen([PathName,FileName],'wt');
% fprintf(fid,'%d\n',hand.pro);
% fclose(fid);
% fid=fopen([PathName,FileName(1:end-4),'_ProfileData.txt'],'wt');
% fprintf(fid,'%d\n',hand.ProfileData);
% fclose(fid);
guidata(hObject, hand);

function D1FitCheck_Callback(hObject, eventdata, hand)
guidata(hObject, hand);
function D2FitCheck_Callback(hObject, eventdata, hand)
guidata(hObject, hand);
function k1FitCheck_Callback(hObject, eventdata, hand)
guidata(hObject, hand);
function k2FitCheck_Callback(hObject, eventdata, hand)
guidata(hObject, hand);


function Fit_Start_Callback(hObject, eventdata, hand)
set(hand.Fit_Start,'BackgroundColor',[1,1,1]);
% Fit_Start=str2double(get(hand.Fit_Start, 'String'));
% tmp = abs(X-Fit_Start*1e-6);
% [val_start hand.Fit_Start_idx] = min(tmp);
guidata(hObject, hand);
function Fit_Start_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function Fit_End_Callback(hObject, eventdata, hand)
set(hand.Fit_End,'BackgroundColor',[1,1,1]);
t1=3600*str2double(get(hand.t1,'String'));
t2=3600*str2double(get(hand.t2,'String'));
t_tot=t1+t2;
D1=str2double(get(hand.D1,'String'));
set(hand.xStar_d,'String',...
    roundsf((str2double(get(hand.Fit_End,'String'))*...
    10^-6)/(2*sqrt(D1*(t_tot ))),3,'round'));
% Fit_End=str2double(get(hand.Fit_End, 'String'));
% tmp = abs(X-Fit_End*1e-6);
% [val_end hand.Fit_End_idx] = min(tmp);

guidata(hObject, hand);
function Fit_End_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function FitRange_Callback(hObject, eventdata, hand)
guidata(hObject, hand);
function FitRange_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function FitPoints_Callback(hObject, eventdata, hand)
guidata(hObject, hand);
function FitPoints_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function D1slider_Callback(hObject, eventdata, hand);
inc=get(hand.D1slider,'Value');
set(hand.D1slider,'Value',0);
D1=str2double(get(hand.D1, 'String'));
% D1=roundsf(D1*(1+inc*0.1),3,'round');
D1=roundsf(D1+inc*10^(floor(log10(D1)-1)),3,'round');
set(hand.D1,'String',num2str(abs(D1)));
if str2double(get(hand.PixelNo,'String'))<1000
    PlotButton_Callback(hObject, eventdata, hand);
end
guidata(hObject, hand);
function D1slider_CreateFcn(hObject, eventdata, hand)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function D2slider_Callback(hObject, eventdata, hand)
inc=get(hand.D2slider,'Value');
set(hand.D2slider,'Value',0);
D2=str2double(get(hand.D2, 'String'));
% D2=roundsf(D2*(1+inc*0.1),3,'round');
D2=roundsf(D2+inc*10^(floor(log10(D2)-1)),3,'round');
set(hand.D2,'String',num2str(abs(D2)));
if str2double(get(hand.PixelNo,'String'))<1000
    PlotButton_Callback(hObject, eventdata, hand);
end
guidata(hObject, hand);
function D2slider_CreateFcn(hObject, eventdata, hand)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function k1slider_Callback(hObject, eventdata, hand)
inc=get(hand.k1slider,'Value');
set(hand.k1slider,'Value',0);
k1=str2double(get(hand.k1, 'String'));
% k1=roundsf(k1*(1+inc*0.1),3,'round');
k1=roundsf(k1+inc*10^(floor(log10(k1)-1)),3,'round');
set(hand.k1,'String',num2str(abs(k1)));
if str2double(get(hand.PixelNo,'String'))<1000
    PlotButton_Callback(hObject, eventdata, hand);
end
guidata(hObject, hand);
function k1slider_CreateFcn(hObject, eventdata, hand)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function k2slider_Callback(hObject, eventdata, hand)
inc=get(hand.k2slider,'Value');
set(hand.k2slider,'Value',0);
k2=str2double(get(hand.k2, 'String'));
% k2=roundsf(k2*(1+inc*0.1),3,'round');
k2=roundsf(k2+inc*10^(floor(log10(k2)-1)),3,'round');
set(hand.k2,'String',num2str(abs(k2)));
if str2double(get(hand.PixelNo,'String'))<1000
    PlotButton_Callback(hObject, eventdata, hand);
end
guidata(hObject, hand);
function k2slider_CreateFcn(hObject, eventdata, hand)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function [hand]=LoadO16image_Callback(hObject, eventdata, hand)
hand.ProLen_Or=500;
hand.O16Flag=1;
if isfield(hand,'FileName')
    [hand.FileName, hand.PathName,~] = ...
        uigetfile([hand.PathName,'*.*'],'Major isotope file');%(FILTERSPEC, TITLE);
else
    [hand.FileName, hand.PathName,~] = ...
        uigetfile('*.*','Major isotope file');%(FILTERSPEC, TITLE);
end
if hand.FileName==0
    return
end
if hand.PathName==0
    hand.O16Flag=0;
else
    [Image]=Loader2D(hand);
    hand.O16image=Image;
    hand.O16image_Or=Image;
    PlotO16image_Callback(hObject, eventdata, hand)
end
if isfield(hand,'O18Flag')
    if hand.O18Flag==1;
        if size(hand.O16image_Or)==size(hand.O18image_Or)
            hand.O16pO18image_Or=hand.O16image_Or+hand.O18image_Or;
            hand.O16pO18image=hand.O16pO18image_Or;
            NormO18image=(hand.O18image./hand.O16pO18image);
            NormO18image(isnan(NormO18image))=0;
            NormO18image(isinf(NormO18image))=0;
            hand.NormO18image_Or=NormO18image;
            hand.NormO18image=hand.NormO18image_Or;
        end
    end
end
guidata(hObject, hand);

function [hand]=LoadO18image_Callback(hObject, eventdata, hand)
hand.CurrentPlot='Image';
hand.O18Flag=1;
if isfield(hand,'FileName')
    [hand.FileName, hand.PathName,~] = ...
        uigetfile([hand.PathName,'*.*'],'Tracer isotope file');%(FILTERSPEC, TITLE);
else
    [hand.FileName, hand.PathName,~] = ...
        uigetfile('*.*','Tracer isotope file');%(FILTERSPEC, TITLE);
end
if hand.FileName==0
    return
end
if hand.PathName==0
    hand.O18Flag=0;
else
    [Image]=Loader2D(hand);
    hand.O18image=Image;
    hand.O18image_Or=Image;
    PlotO18image_Callback(hObject, eventdata, hand)
end
if isfield(hand,'O16Flag')
    if hand.O16Flag==1;
        if size(hand.O16image_Or)==size(hand.O18image_Or)
            hand.O16pO18image_Or=hand.O16image_Or+hand.O18image_Or;
            hand.O16pO18image=hand.O16pO18image_Or;
            NormO18image=(hand.O18image./hand.O16pO18image);
            NormO18image(isnan(NormO18image))=0;
            NormO18image(isinf(NormO18image))=0;
            hand.NormO18image_Or=NormO18image;
            hand.NormO18image=hand.NormO18image_Or;
        end
    end
end
hand.SaveName_Or=[hand.FileName(1:end-4),...
    '_Norm18Prof.txt'];
set(hand.ProfDataSaveName, 'String',hand.FileName);
guidata(hObject, hand);

function [Image]=Loader2D(hand)
ext=hand.FileName(find(hand.FileName=='.',1,'last'):end);
if ext=='.txt'
    if ispc
        Image=dlmread([hand.PathName,'\',hand.FileName], ' ', 9, 2);
    else
        Image=dlmread([hand.PathName,hand.FileName], ' ', 9, 2);
    end
    PixelNo = sqrt(length(Image));
    set(hand.PixelNo,'String',num2str(PixelNo));
    Image=transpose(reshape(Image,PixelNo,PixelNo));
    
    %%%%%%%%%%%%%%%%%
    if ispc
        fileID = fopen([hand.PathName,'\',hand.FileName]);
    else
        fileID = fopen([hand.PathName,hand.FileName]);
    end
    LenStr=textscan(fileID,'%s',4,'Delimiter','\n');
    LenStr=LenStr{1,1}{3,1};
    LenStr=roundsf(str2double(LenStr(18:30)),3,'round');
    fclose(fileID);
    set(hand.ProfileLength,'String',LenStr);
    set(hand.Fit_End,'String',roundsf(...
        0.5*str2double(get(hand.ProfileLength,'String')),3,'floor'));
    set(hand.ProfileLength,'BackgroundColor',[0,1,0]);
    set(hand.Xlimit,'String',LenStr);
    %     ProfileLength=roundsf(dlmread([hand.PathName,'\',...
    %         hand.FileName], ':', [2 1 2 1]),4,'round');
    %     set(hand.ProfileLength,'String',ProfileLength);
    %     set(hand.ProfileLength,'BackgroundColor',[0,1,0]);
    %     set(hand.Xlimit,'String',ProfileLength);
    
    
    % elseif ext=='.tif' %otherwise people will think this is data
    %     Image=imread([hand.PathName,'\',hand.FileName]);
    %     Image=double(Image);
    %     PixelNo=length(Image);
elseif ext=='.ext'
    fileID = fopen([hand.PathName,hand.FileName]);
    tline = fgets(fileID);
    C = textscan(fileID,'%f','HeaderLines',1);
    H=textscan(tline, '%s %s %s %f %s %f %f','Delimiter',' ');
    rows=H{1,4};
    Image=vec2mat(C{1,1},rows);
    Image=double(Image);
    PixelNo=length(Image);
    fclose(fileID);
end
set(hand.PixelNo,'String',PixelNo);
set(hand.PixelNo,'BackgroundColor',[0,1,0]);

function PlotO16image_Callback(hObject, eventdata, hand)
hand.CurrentPlot='2Dmap';
set(hand.Xlimit,'Visible','off');
set(hand.Ylimit,'Visible','off');
if ~isfield(hand,'O16Flag')
    [hand]=LoadO16image_Callback(hObject, eventdata, hand);
end
% axes(hand.MainAxes)
imagesc(hand.O16image_Or);colormap(hot);
hand.colo=colorbar; ylabel(hand.colo,'Counts');
axis square; set(gca, 'XTick', [],'YTick', []);

% set(gca,'position',[90    60   465   465]); %pixels
PlotProperties(hObject, eventdata, hand)
set(gca,'position',hand.ImagePlotPos); %norm

function PlotO18image_Callback(hObject, eventdata, hand)
hand.CurrentPlot='2Dmap';
set(hand.Xlimit,'Visible','off');
set(hand.Ylimit,'Visible','off');
if ~isfield(hand,'O18Flag')
    [hand]=LoadO18image_Callback(hObject, eventdata, hand);
end
% axes(hand.MainAxes)
imagesc(hand.O18image_Or);colormap(hot);colo=colorbar;ylabel(colo,'Counts'); axis square; set(gca, 'XTick', [],'YTick', []);
% set(gca,'position',[90    60   465   465]);%pix
PlotProperties(hObject, eventdata, hand)
set(gca,'position',hand.ImagePlotPos); %norm

function [hand]=PlotO16pO18image_Callback(hObject, eventdata, hand)
hand.CurrentPlot='2Dmap';
set(hand.Xlimit,'Visible','off');
set(hand.Ylimit,'Visible','off');
if ~isfield(hand,'O16Flag')
    [hand]=LoadO16image_Callback(hObject, eventdata, hand);
else
    if hand.O16Flag==0
        [hand]=LoadO16image_Callback(hObject, eventdata, hand);
    end
end
if ~isfield(hand,'O18Flag')
    [hand]=LoadO18image_Callback(hObject, eventdata, hand);
else
    if hand.O18Flag==0
        [hand]=LoadO18image_Callback(hObject, eventdata, hand);
    end
end
% axes(hand.MainAxes)
imagesc(hand.O16pO18image_Or);colormap(hot);colo=colorbar;ylabel(colo,'Counts'); axis square; set(gca, 'XTick', [],'YTick', []);
%hand.N_O18image=hand.O18image./hand.O16pO18image;
%imagesc((hand.N_O18image));colormap(hot);colo=colorbar;ylabel(colo,'Counts'); axis square; set(gca, 'XTick', [],'YTick', []);
% set(gca,'position',[90    60   465   465]);%pix
set(gca,'position',hand.ImagePlotPos); %norm
PlotProperties(hObject, eventdata, hand)
guidata(hObject, hand);

function PlotNormO18image_Callback(hObject, eventdata, hand)
hand.CurrentPlot='2Dmap_norm';
set(hand.Xlimit,'Visible','off');
set(hand.Ylimit,'Visible','off');
if ~isfield(hand,'O16Flag')
    [hand]=LoadO16image_Callback(hObject, eventdata, hand);
else
    if hand.O16Flag==0
        [hand]=LoadO16image_Callback(hObject, eventdata, hand);
    end
end
if ~isfield(hand,'O18Flag')
    [hand]=LoadO18image_Callback(hObject, eventdata, hand);
else
    if hand.O18Flag==0
        [hand]=LoadO18image_Callback(hObject, eventdata, hand);
    end
end
% axes(hand.MainAxes)
imagesc(hand.NormO18image_Or);colormap(hot);colo=colorbar;ylabel(colo,'Isotopic Fraction'); axis square; set(gca, 'XTick', [],'YTick', []);
%hand.N_O18image=hand.O18image./hand.O16pO18image;
%imagesc((hand.N_O18image));colormap(hot);colo=colorbar;ylabel(colo,'Counts'); axis square; set(gca, 'XTick', [],'YTick', []);

% set(gca,'position',[90    60   465   465]); %pixels
set(gca,'position',hand.ImagePlotPos); %norm
PlotProperties(hObject, eventdata, hand)
guidata(hObject, hand);

function [hand]=Align_Callback(hObject, eventdata, hand)
hand.CurrentPlot='Align';
hand.PlotType=1;
hand.Aligning=1;
AlignMode=get(hand.AlignMode, 'Value');
if length(get(hand.Advanced,'Visible'))==2
    Go=1;
else
    Go=0;
end
% set(hand.Xlimit,'Visible','off');
if ~isfield(hand,'O16Flag')
    [hand]=PlotO16pO18image_Callback(hObject, eventdata, hand);
end
hand.AlignFlag=1;
set(hand.Align,'ForegroundColor',[0.5,0.5,0.5]);
% set(hand.Align,'Enable','off'); %Don't turn it off, just make it grey
set(hand.Align,'String','Aligning...');pause(0.001);

[a,b]=size(hand.O16pO18image_Or);
Mask_Or=ones(a,b);
CoarseStep=10;
for theta=0:CoarseStep:360-CoarseStep
    i=1+theta/CoarseStep;
    try
        O16pO18image_r=imrotate(hand.O16pO18image_Or,theta,'bilinear','crop'); %,'crop'
        mask_r=imrotate(Mask_Or,theta,'nearest','crop');
    catch ME
        O16pO18image_r=imrotate2(hand.O16pO18image_Or,theta,'bilinear','crop'); %,'crop'
        mask_r=imrotate2(Mask_Or,theta,'nearest','crop');
    end
    
    sum_mod=sum(O16pO18image_r)./sum(mask_r);
    if Go==1
        sumlog_mod=sum(log(O16pO18image_r))./sum(mask_r);
        logsum_mod=log(sum(O16pO18image_r)./sum(mask_r));
        Max_LogGrad_C(i)=max(diff(logsum_mod));
        Sum_LogGrad_C(i)=sum(diff(log(sum_mod)));
        minsum(i)=min(sum_mod);
        Max_Grad_C(i)=max(diff(sum_mod));
        Sum_Grad_C(i)=sum(diff(sum_mod));
    elseif AlignMode==5
        sumlog_mod=sum(log(O16pO18image_r))./sum(mask_r);
        logsum_mod=log(sum(O16pO18image_r)./sum(mask_r));
        Max_LogGrad_C(i)=max(diff(logsum_mod));
        Sum_LogGrad_C(i)=sum(diff(log(sum_mod)));
    else
        minsum(i)=min(sum_mod);
        Max_Grad_C(i)=max(diff(sum_mod));
        Sum_Grad_C(i)=sum(diff(sum_mod));
    end
    
end
Theta_C=[0:CoarseStep:360-CoarseStep];

%% Go mode plotting
if length(get(hand.Advanced,'Visible'))==2
    set(hand.Xlimit,'Visible','off');
    %     a = annotation(gcf,'textbox',...
    %         [0.35 0.6 0.2 0.12],...
    %         'String','Press enter to continue',...
    %         'FontSize',20);
    %     plot(Theta_C,medfilt2(Max_Grad_C,[1 3])/max(medfilt2(Max_Grad_C,[1 3])),...
    %         Theta_C,Sum_Grad_C/max(Sum_Grad_C));
    %     legend('medfil(Max. Grad.)','Total Grad.');
    %     set(gca,'position',hand.GraphPlotPos);
    %     pause;
    plot(Theta_C,Max_Grad_C/max(Max_Grad_C),...
        Theta_C, minsum./max(minsum),...
        Theta_C,Sum_Grad_C/max(Sum_Grad_C),...
        Theta_C,Max_Grad_C.*Sum_Grad_C/(max(Max_Grad_C)*max(Sum_Grad_C)),...
        Theta_C,Max_LogGrad_C./max(Max_LogGrad_C(isfinite(Max_LogGrad_C))),...
        Theta_C,Sum_LogGrad_C./max(Sum_LogGrad_C(isfinite(Sum_LogGrad_C))));
    legend('Max. Grad.','MinSum','Total Grad.','Combo. Grad','Max LogGrad_C','Sum LogGrad_C');
    xlabel('Angle');set(gca,'position',hand.GraphPlotPos);
    %     f = warndlg('This is a warning.', 'A Warning Dialog');
    %     waitfor(f);
    pause(.1);
    %     delete(a);
    set(hand.Xlimit,'Visible','on');
end

%% Back to things
switch AlignMode
    case 1
        [~, CoarseMaxVal_idx]=max(Max_Grad_C.*Sum_Grad_C);
    case 2
        [~, CoarseMaxVal_idx]=max(Max_Grad_C);
    case 3
        [~, CoarseMaxVal_idx]=max(Sum_Grad_C);
    case 4
        [~, CoarseMaxVal_idx]=min(minsum);
    case 5
        [~, CoarseMaxVal_idx]=max(Max_LogGrad_C);
end
FA_init=Theta_C(CoarseMaxVal_idx);
% FA_init=(CoarseMaxVal_idx-1)*CoarseStep;
i=1;
AngleStep_Fine=0.2;
Wind=40;
Theta_F  =FA_init-Wind/2 : AngleStep_Fine : FA_init+Wind/2;
for theta=FA_init-Wind/2 : AngleStep_Fine : FA_init+Wind/2
    try
        O16pO18image_r=imrotate(hand.O16pO18image_Or,theta,'bilinear','crop'); %,'crop'
        mask_r=imrotate(Mask_Or,theta,'nearest','crop');
    catch ME
        O16pO18image_r=imrotate2(hand.O16pO18image_Or,theta,'bilinear','crop'); %,'crop'
        mask_r=imrotate2(Mask_Or,theta,'nearest','crop');
    end
    sum_mod=sum(O16pO18image_r)./sum(mask_r);
    minsum(i)=min(sum_mod);
    [Max_Grad_F(i,1), Max_Grad_F(i,2)]=max(diff(sum_mod));
    Sum_Grad_F(i)=sum(diff(sum_mod));
    
    
    %     sumlog_mod=sum(log(O16pO18image_r))./sum(mask_r);
    logsum_mod=log(sum(O16pO18image_r)./sum(mask_r));
    Max_LogGrad_F(i)=max(diff(logsum_mod));
    Sum_LogGrad_F(i)=sum(diff(log(sum_mod)));
    i=i+1;
end
%% Go mode plotting
if length(get(hand.Advanced,'Visible'))==2
    set(hand.Xlimit,'Visible','off');
    a = annotation(gcf,'textbox',...
        [0.35 0.6 0.2 0.12],...
        'String','Press enter to continue',...
        'FontSize',20);
    plot(Theta_F,Max_Grad_F(:,1)/max(Max_Grad_F(:,1)),...
        Theta_F, minsum./max(minsum),...
        Theta_F, Sum_Grad_F/max(Sum_Grad_F),...
        Theta_F,Sum_Grad_F'.*Max_Grad_F(:,1)/(max(Sum_Grad_F*max(Max_Grad_F(:,1)))),...
        Theta_F,Max_LogGrad_F./max(Max_LogGrad_F(isfinite(Max_LogGrad_F))),...
        Theta_F,Sum_LogGrad_F./max(Sum_LogGrad_F(isfinite(Sum_LogGrad_F))));
    legend('Max. Grad.','MinSum','Total Grad.','Combo. Grad','Max LogGrad','Sum LogGrad');
    xlabel('Angle');
    hand.GraphPlotPos=[.07 .09 .67 .71];%norm
    %     plot(Theta_C,Max_Grad_C/max(Max_Grad_C),...
    %         Theta_C, minsum./max(minsum),...
    %         Theta_C,Sum_Grad_C/max(Sum_Grad_C),...
    %         Theta_C,Max_Grad_C.*Sum_Grad_C/(max(Max_Grad_C)*max(Sum_Grad_C)),...
    %         Theta_C,Max_LogGrad_C./max(Max_LogGrad_C(isfinite(Max_LogGrad_C))),...
    %         Theta_C,Sum_LogGrad_C./max(Sum_LogGrad_C(isfinite(Sum_LogGrad_C))));
    %     legend('Max. Grad.','MinSum','Total Grad.','Combo. Grad','Max LogGrad_C','Sum LogGrad_C');
    %     xlabel('Angle');
    delete(a)
    set(hand.Xlimit,'Visible','on');
    %
end

%% Back to align
switch AlignMode
    case 1;
        [Max_Grad_F_val Max_Grad_F_idx]=max(Sum_Grad_F'.*Max_Grad_F(:,1));
    case 2;
        [Max_Grad_F_val Max_Grad_F_idx]=max(Max_Grad_F(:,1));
    case 3;
        [Max_Grad_F_val Max_Grad_F_idx]=max(Sum_Grad_F);%Will also tell you where the surf is
    case 4;
        [Max_Grad_F_val Max_Grad_F_idx]=min(minsum);
    case 5
        [Max_Grad_F_val Max_Grad_F_idx]=max(Max_LogGrad_F);
end

AlignAngle=Theta_F(Max_Grad_F_idx);

hand.AlignAngle_FiAl_num=AlignAngle;
hand.AlignAngle_CoAl_num=round(AlignAngle/90)*90;
% hand.Mask_FiAl_Basic=imrotate(Mask_Or,AlignAngle,'nearest','crop');
% hand.Mask_CoAl_Basic=imrotate(Mask_Or,hand.AlignAngle_CoAl_num,'nearest','crop');

set(hand.AlignAngle,'String',num2str(AlignAngle));
[hand]=AlignAngle_Callback(hObject, eventdata, hand);
set(hand.AlignAngle,'BackgroundColor',[0 1 0]);
set(hand.Align,'ForegroundColor',[0,0,0]);
set(hand.Align,'String','Align');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% switch AlignMode
%     case 1
%         fun = @(theta)...
%             -sum(diff(...
%             sum(imrotate(hand.O16pO18image_Or,theta,'bilinear','crop'))./...
%             sum(imrotate(Mask_Or,theta,'nearest','crop'))))...
%             *max(diff(...
%             sum(imrotate(hand.O16pO18image_Or,theta,'bilinear','crop'))./...
%             sum(imrotate(Mask_Or,theta,'nearest','crop'))));
%         [theta,fminres] = fminbnd(fun,0,360);%,optimset('TolFun',1e-2,'TolX',1e-1));
%         theta
%     case 2
%         fun = @(theta)...
%             double(-max(diff(...
%             sum(imrotate(hand.O16pO18image_Or,theta,'bilinear','crop'))./...
%             sum(imrotate(Mask_Or,theta,'nearest','crop')))));
%         [theta,fminres] = fminbnd(fun,0,360);%,optimset('TolFun',1e-2,'TolX',1e-2));
%         theta
%
%         opts = optimoptions(@fmincon,'Algorithm','interior-point');
%         problem = createOptimProblem('fmincon','objective',...
%             @(theta) fun,'x0',3,'lb',0,'ub',360,'options',opts);
%         gs = GlobalSearch;
%         [x,f] = run(gs,problem)
%
%     case 3
%         fun = @(theta)...
%             -sum(diff(...
%             sum(imrotate(hand.O16pO18image_Or,theta,'bilinear','crop'))./...
%             sum(imrotate(Mask_Or,theta,'nearest','crop'))));
%         [theta,fminres] = fminbnd(fun,0,360);%,optimset('TolFun',1e-2,'TolX',1e-2));
%         theta
%     case 4
%     case 5
%         fun = @(theta)...
%             -max(diff(...
%             log(sum(imrotate(hand.O16pO18image_Or,theta,'bilinear','crop')))./...
%             sum(imrotate(Mask_Or,theta,'nearest','crop'))));
%         [theta,fminres] = fminbnd(fun,0,360);%,optimset('TolFun',1e-2,'TolX',1e-2));
%         theta
% end


guidata(hObject, hand);

function AlignMode_Callback(hObject, eventdata, hand)
[hand]=Align_Callback(hObject, eventdata, hand);
PlotProperties(hObject, eventdata, hand)
guidata(hObject, hand);
function AlignMode_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function [hand]=AlignAngle_Callback(hObject, eventdata, hand)
hand.CurrentPlot='Align';
if ~isfield(hand,'AlignAngle_CoAl_num')
    hand.AlignAngle_CoAl_num=0;
end
set(hand.Ylimit,'Visible','off');
hand.AlignAngle_FiAl_num=str2num(get(hand.AlignAngle,'String'));
set(hand.AlignAngle,'BackgroundColor',[1 1 1]);
set(gca,'position',hand.ImagePlotPos);
if isempty(hand.AlignAngle_FiAl_num)
    hand.AlignAngle_FiAl_num=0;
end
%%% Rotate all images
try
    hand.O16image_FiAl=imrotate(hand.O16image_Or,...
        hand.AlignAngle_FiAl_num,'bilinear','crop');
    hand.O18image_FiAl=imrotate(hand.O18image_Or,...
        hand.AlignAngle_FiAl_num,'bilinear','crop');
    hand.O16pO18image_FiAl=imrotate(hand.O16pO18image_Or,...
        hand.AlignAngle_FiAl_num,'bilinear','crop');
    hand.NormO18image_FiAl=imrotate(hand.NormO18image_Or,...
        hand.AlignAngle_FiAl_num,'bilinear','crop');
catch ME
    hand.O16image_FiAl=imrotate2(hand.O16image_Or,...
        hand.AlignAngle_FiAl_num,'bilinear','crop');
    hand.O18image_FiAl=imrotate2(hand.O18image_Or,...
        hand.AlignAngle_FiAl_num,'bilinear','crop');
    hand.O16pO18image_FiAl=imrotate2(hand.O16pO18image_Or,...
        hand.AlignAngle_FiAl_num,'bilinear','crop');
    hand.NormO18image_FiAl=imrotate2(hand.NormO18image_Or,...
        hand.AlignAngle_FiAl_num,'bilinear','crop');
end
% hand.O16pO18image_FiAl=imrotate(hand.O16pO18image_Or,hand.AlignAngle_FiAl_num,'bicubic','crop');

%%% Coarse align all images if possible
if isfield(hand,'AlignAngle_CoAl_num')
    try
        hand.O16image_CoAl=imrotate(hand.O16image_Or,...
            hand.AlignAngle_CoAl_num,'bilinear','crop');
        hand.O18image_CoAl=imrotate(hand.O18image_Or,...
            hand.AlignAngle_CoAl_num,'bilinear','crop');
        hand.O16pO18image_CoAl=imrotate(hand.O16pO18image_Or,...
            hand.AlignAngle_CoAl_num,'bilinear','crop');
        hand.NormO18image_CoAl=imrotate(hand.NormO18image_Or,...
            hand.AlignAngle_CoAl_num,'bilinear','crop');
    catch ME
        hand.O16image_CoAl=imrotate2(hand.O16image_Or,...
            hand.AlignAngle_CoAl_num,'bilinear','crop');
        hand.O18image_CoAl=imrotate2(hand.O18image_Or,...
            hand.AlignAngle_CoAl_num,'bilinear','crop');
        hand.O16pO18image_CoAl=imrotate2(hand.O16pO18image_Or,...
            hand.AlignAngle_CoAl_num,'bilinear','crop');
        hand.NormO18image_CoAl=imrotate2(hand.NormO18image_Or,...
            hand.AlignAngle_CoAl_num,'bilinear','crop');
    end
end

%%% Rotate all masks
[a,b]=size(hand.O16pO18image_Or);
Mask_Or=ones(a,b);
try
    hand.Mask_FiAl_Basic=imrotate(Mask_Or,...
        hand.AlignAngle_FiAl_num,'nearest','crop');
    hand.Mask_CoAl_Basic=imrotate(Mask_Or,...
        hand.AlignAngle_CoAl_num,'nearest','crop');
catch ME
    hand.Mask_FiAl_Basic=imrotate2(Mask_Or,...
        hand.AlignAngle_FiAl_num,'nearest','crop');
    hand.Mask_CoAl_Basic=imrotate2(Mask_Or,...
        hand.AlignAngle_CoAl_num,'nearest','crop');
end
hand.counts=sum(hand.O16pO18image_FiAl)./sum(hand.Mask_FiAl_Basic);
hand.counts(hand.counts==inf)=nan;
hand.counts(hand.counts==-inf)=nan;
%%% Plot fine align
% imagesc(hand.O16pO18image_FiAl);
% colormap(hot); axis square;set(gca, 'XTick', [],'YTick', []);
% set(gca,'position',[68    62   500   465]);
logCounts=log10(hand.counts);

ProfileLength=str2double(get(hand.ProfileLength, 'String'));
PixelNo=str2double(get(hand.PixelNo, 'String'));
DataStep=ProfileLength*1e-6/(PixelNo-1); %spatial step
hand.X=0:DataStep:ProfileLength*1e-6; %domain

if get(hand.AlignMode, 'Value')==5
    imagesc([0 ProfileLength],...
        [floor(min(log10(hand.counts(isfinite(log10(hand.counts)))))),...
        ceil(max(log10(hand.counts(isfinite(log10(hand.counts))))))],...
        flipud(log10(hand.O16pO18image_FiAl))); axis square;
    hold on;
    
    plot(hand.X*1e6,log10(hand.counts),'c','linewidth',2.5);
    hold off
    hand.colo=colorbar; ylabel(hand.colo,'Log(Counts)');
    legend('Log(Mean Counts)');
    
    if isfinite(sum(logCounts))
        finitecountstart=2;
    else
        finitecountstart=2+find(~isfinite(logCounts),1,'last');
    end
    [SurfGrad SurfPos_idx]=max(diff(logCounts(finitecountstart:end/2)));
    %
    SurfPos_idx=finitecountstart+SurfPos_idx;
    ylabel('Log(Mean Counts (^{16}O + ^{18}O))');
else
    imagesc([0 ProfileLength],[0 ceil(max(hand.counts))],...
        flipud(hand.O16pO18image_FiAl)); axis square;
    hold on;
    plot(hand.X*1e6,hand.counts,'c','linewidth',2.5);
    hold off
    hand.colo=colorbar; ylabel(hand.colo,'Counts');
    legend('Mean Counts');
    
    [SurfGrad SurfPos_idx]=max(diff(hand.counts(1:end/2)));
    
    SurfPos_idx=1+SurfPos_idx;
    if isfield(hand,'Secrets')
        if hand.Secrets>=4
            if str2double(get(hand.SurfPos,'String'))==0
                SurfPos_idx=1;
            end
        end
    end
    
    ylabel('Mean Counts (^{16}O + ^{18}O)');
end

set(gca,'ydir','normal');
xlabel('Depth /\mum');

xlim([0 str2double(get(hand.Xlimit,'String'))]);
colormap(hot); axis square;
set(hand.Xlimit,'Visible','on');
set(hand.Xlimit,'Position',hand.ImagePlotPosXlim);


hand.SurfPos_idx=SurfPos_idx; %%% the gradient is between these two locations, we want to be on the high counts side
SurfPos=1e6*hand.X(SurfPos_idx);
set(hand.SurfPos,'String',...
    num2str(roundsf(SurfPos,3,'round')));
set(hand.SurfPos,'BackgroundColor',[0 1 0]);

hand.SurfLine2=line([SurfPos, SurfPos],[-5000 5000],...
    'LineWidth',2,'Color',[.1 1 .1],'LineStyle',':');
% hand.SurfLine=imline(gca,[SurfPos, SurfPos], [-5000, 5000]);%,'Color','w','LineStyle','--');
% hand.h1 = get(hand.SurfLine,'Children');
% set(hand.h1,'LineStyle',':');
% set(hand.h1,'LineWidth',2);
% setColor(hand.SurfLine,[0.1 1 0.1]);
% pos = hand.SurfLine.getPosition();
PlotProperties(hObject, eventdata, hand)
pause(0.1)
guidata(hObject, hand);

function AlignAngle_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function MaskMode_Callback(hObject, eventdata, hand)
switch get(hand.MaskMode, 'Value')
    case 1;
        set(hand.MaskThresh,'String','0.5')
    case 2;
        set(hand.MaskThresh,'String','1')
end
% MaskButton_Callback(hObject, eventdata, hand);
guidata(hObject, hand);
function MaskMode_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function MaskThresh_Callback(hObject, eventdata, hand)
MaskButton_Callback(hObject, eventdata, hand);
guidata(hObject, hand);
function MaskThresh_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function [hand]=MaskButton_Callback(hObject, eventdata, hand)
hand.CurrentPlot='Mask';
hand.PlotType=1;
hand.ROIflag=0;
hand.Aligning=0;
if ~isfield(hand,'AlignAngle_FiAl_num')
    %     [hand]=Align_Callback(hObject, eventdata, hand);
    if ~isfield(hand,'O16Flag')
        [hand]=PlotO16pO18image_Callback(hObject, eventdata, hand);
    end
    hand.AlignAngle_FiAl_num=0;
    hand.AlignAngle_CoAl_num=0;
    hand.O16image_FiAl=hand.O16image;
    hand.O18image_FiAl=hand.O18image;
    hand.O16image_CoAl=hand.O16image;
    hand.O18image_CoAl=hand.O18image;
    hand.O16pO18image_FiAl=hand.O16pO18image_Or;
    
    hand.O16pO18image_CoAl=hand.O16pO18image_Or;
    
    
    [a,b]=size(hand.O16pO18image_Or);
    Mask_Or=ones(a,b);
    hand.Mask_FiAl_Basic=Mask_Or;
    hand.Mask_CoAl_Thresh=Mask_Or;
    hand.Mask_CoAl_Basic=Mask_Or;
    %     hand.O16pO18image_Or=
    %     hand.Mask_FiAl_Thresh=
end
set(hand.Xlimit,'Visible','on');
set(hand.Xlimit,'Position',hand.ImagePlotPosXlim);
set(hand.Ylimit,'Visible','off');
[a,b]=size(hand.O16pO18image_Or);
Mask_Or=ones(a,b);
% Mask_FiAl_Basic=hand.Mask_FiAl_Basic;
% Mask_CoAl_Basic=hand.Mask_CoAl_Basic;

MaskStd=std(hand.O16pO18image_Or(hand.O16pO18image_Or~=0));
MaskMean=mean(hand.O16pO18image_Or(hand.O16pO18image_Or~=0));

switch get(hand.MaskMode, 'Value')
    case 1;
        Threshold=MaskMean*str2double(get(hand.MaskThresh,'String'));
        Mask_Or(hand.O16pO18image_Or<Threshold)=0;
        try
            hand.Mask_FiAl_Thresh=imrotate(Mask_Or,hand.AlignAngle_FiAl_num,...
                'nearest','crop');
            hand.Mask_CoAl_Thresh=imrotate(Mask_Or,hand.AlignAngle_CoAl_num,...
                'nearest','crop');
        catch ME
            hand.Mask_FiAl_Thresh=imrotate2(Mask_Or,hand.AlignAngle_FiAl_num,...
                'nearest','crop');
            hand.Mask_CoAl_Thresh=imrotate2(Mask_Or,hand.AlignAngle_CoAl_num,...
                'nearest','crop');
        end
    case 2;
        Threshold_L=MaskMean-...
            str2double(get(hand.MaskThresh,'String'))*MaskStd;
        Threshold_H=MaskMean+...
            str2double(get(hand.MaskThresh,'String'))*MaskStd;
        % sum(sum(hand.O16pO18image~=0))
        Mask_Or(hand.O16pO18image_Or<Threshold_L | ...
            hand.O16pO18image_Or>Threshold_H)=0;
        try
            hand.Mask_FiAl_Thresh=imrotate(Mask_Or,...
                hand.AlignAngle_FiAl_num,'nearest','crop');
            hand.Mask_CoAl_Thresh=imrotate(Mask_Or,...
                hand.AlignAngle_CoAl_num,'nearest','crop');
        catch ME
            hand.Mask_FiAl_Thresh=imrotate2(Mask_Or,...
                hand.AlignAngle_FiAl_num,'nearest','crop');
            hand.Mask_CoAl_Thresh=imrotate2(Mask_Or,...
                hand.AlignAngle_CoAl_num,'nearest','crop');
        end
    case 3;
        %%% Where the norm18 in a column is more than 1 sd away from mean
        %%% of that column
end


ProLen=str2double(get(hand.ProfileLength,'String'));
x=[0,ProLen];
% set(gca,'position',[90    60   465   465]);%pix
set(gca,'position',hand.ImagePlotPos); %norm

%%% Plot mask
imagesc(x,x,hand.Mask_FiAl_Thresh);
set(gca, 'YTick', [])
xlim([0 str2double(get(hand.Xlimit,'String'))]);
colormap(hot); axis square;xlabel('Depth /\mum');
PlotProperties(hObject, eventdata, hand)
%%% Show surface line
SurfPos=str2double(get(hand.SurfPos,'String'));
hand.SurfLine2=line([SurfPos, SurfPos],[-5000 5000],...
    'LineWidth',2,'Color',[1 0 0],'LineStyle',':');
% hand.SurfLine=imline(gca,[SurfPos, SurfPos], [-5000, 5000]);%,'Color','w','LineStyle','--');
% hand.h1 = get(hand.SurfLine,'Children');
% set(hand.h1,'LineStyle',':');
% set(hand.h1,'LineWidth',2);
% setColor(hand.SurfLine,[1 0 0]);
pause(0.4)

%% Plot masked image
imagesc(x,x,hand.O16pO18image_FiAl.*hand.Mask_FiAl_Thresh);
set(gca, 'YTick', [])
xlim([0 str2double(get(hand.Xlimit,'String'))]);
colormap(hot); axis square;xlabel('Depth /\mum');% set(gca, 'YTick', [])
if isfield(hand,'SurfLine2')
    delete(hand.SurfLine2)
end
hand.SurfLine2=line([SurfPos, SurfPos],[-5000 5000],...
    'LineWidth',2,'Color',[.1 1 .1],'LineStyle',':');
% hand.SurfLine=imline(gca,[SurfPos, SurfPos], [-5000, 5000]);%,'Color','w','LineStyle','--');
% hand.h1 = get(hand.SurfLine,'Children');
% set(hand.h1,'LineStyle',':');
% set(hand.h1,'LineWidth',2);
% setColor(hand.SurfLine,[0.1 1 0.1]);

ProfileLength=str2double(get(hand.ProfileLength, 'String'));
PixelNo=str2double(get(hand.PixelNo, 'String'));
dx=ProfileLength*1e-6/(PixelNo-1); %spatial step
hand.X=0:dx:ProfileLength*1e-6; %domain

% grid (get(hand.GridLines,'State'))
PlotProperties(hObject, eventdata, hand)
guidata(hObject, hand);




function ROI_Callback(hObject, eventdata, hand)
[hand]=MaskButton_Callback(hObject, eventdata, hand);
hand.ROIflag=1;
%%% make variables
ProfileLength=str2double(get(hand.ProfileLength, 'String'));
SurfPos=str2double(get(hand.SurfPos,'String'));

%%%spec ROI
[x,y,BW,xi,yi] = roipoly;
hand.ROImin=min(xi);%Know to change fit limits
hand.ROImax=max(xi);
hand.Mask_FiAl_Thresh=hand.Mask_FiAl_Thresh.*BW;
%%%plot ROI mask
imagesc(hand.X*1e6,hand.X*1e6,hand.Mask_FiAl_Thresh);
xlim([0 str2double(get(hand.Xlimit,'String'))]);
set(gca, 'YTick', []); colormap(hot);
axis square; xlabel('Depth /\mum');
% hand.SurfLine=imline(gca,[SurfPos, SurfPos], [-5000, 5000]);%,'Color','w','LineStyle','--');
% hand.h1 = get(hand.SurfLine,'Children');
% set(hand.h1,'LineStyle',':');
% set(hand.h1,'LineWidth',2);
% setColor(hand.SurfLine,[1 0 0]);
hand.SurfLine2=line([SurfPos, SurfPos],[-5000 5000],...
    'LineWidth',2,'Color',[1 0 0],'LineStyle',':');

pause(0.5)

%%%plot thresholded region
imagesc([0 ProfileLength],[0 ProfileLength],hand.O16pO18image_FiAl.*hand.Mask_FiAl_Thresh);
xlim([0 str2double(get(hand.Xlimit,'String'))]);
set(gca, 'YTick', []); colormap(hot);
axis square;xlabel('Depth /\mum');% set(gca, 'YTick', [])
% hand.SurfLine=imline(gca,[SurfPos, SurfPos], [-5000, 5000]);%,'Color','w','LineStyle','--');
% hand.h1 = get(hand.SurfLine,'Children');
% set(hand.h1,'LineStyle',':');
% set(hand.h1,'LineWidth',2);
% setColor(hand.SurfLine,[0.1 1 0.1]);
if isfield(hand,'SurfLine2')
    delete(hand.SurfLine2)
end
hand.SurfLine2=line([SurfPos, SurfPos],[-5000 5000],...
    'LineWidth',2,'Color',[.1 1 .1],'LineStyle',':');
PlotProperties(hObject, eventdata, hand)
guidata(hObject, hand);

function GenerateProfiles_Callback(hObject, eventdata, hand)
hand.PlotType=1;
hand.CurrentPlot='Generate';
if ~isfield(hand,'Mask_FiAl_Thresh')
    [hand]=MaskButton_Callback(hObject, eventdata, hand);
else
    [hand]=MaskButton_Callback(hObject, eventdata, hand);
end

Mask_FiAl_Thresh=hand.Mask_FiAl_Thresh; %Mask

%
NormO18image_FiAl_MaTh=hand.O18image_FiAl./hand.O16pO18image_FiAl.*Mask_FiAl_Thresh;
NormO18image_FiAl_MaTh(isnan(NormO18image_FiAl_MaTh))=0;
NormO18image_FiAl_MaTh(isinf(NormO18image_FiAl_MaTh))=0;
hand.NormO18image_FiAl_MaTh=NormO18image_FiAl_MaTh;

O16pO18prof_FiAl_MaTh=sum(hand.O16pO18image_FiAl.*hand.Mask_FiAl_Thresh)./sum(hand.Mask_FiAl_Thresh); %Masked O16pO18
% O16pO18prof_FiAl_MaTh(isnan(O16pO18prof_FiAl_MaTh))=0;
O16pO18prof_FiAl_MaTh(isinf(O16pO18prof_FiAl_MaTh))=0;
O18prof_FiAl_MaTh=sum(hand.O18image_FiAl.*hand.Mask_FiAl_Thresh)./sum(hand.Mask_FiAl_Thresh); %Masked O18
% O18prof_FiAl_MaTh(isnan(O18prof_FiAl_MaTh))=0;
O18prof_FiAl_MaTh(isinf(O18prof_FiAl_MaTh))=0;
% O16prof_FiAl_MaTh=sum(hand.O16image_FiAl.*hand.Mask_FiAl_Thresh)./sum(hand.Mask_FiAl_Thresh); %Masked O18
% O16prof_FiAl_MaTh(isnan(O16prof_FiAl_MaTh))=0;
% O16prof_FiAl_MaTh(isinf(O16prof_FiAl_MaTh))=0;

% UserProfSpec=1;
NormO18prof_FiAl_MaTh=O18prof_FiAl_MaTh./O16pO18prof_FiAl_MaTh; %Normalised profile
P(1,:)=NormO18prof_FiAl_MaTh;

%%% Crap profiles

%Profile based Aligned not masked
% O16pO18prof_AnM=sum(hand.O16pO18image);%.*hand.Mask_FiAl_Basic); %Masked O16pO18
% O18prof_AnM=sum(hand.O18image);%.*hand.Mask_FiAl_Basic);
% NormO18prof_AnM=O18prof_AnM./O16pO18prof_AnM;
% NormO18prof_AnM(isnan(NormO18prof_AnM))=nan;
% NormO18prof_AnM(isinf(NormO18prof_AnM))=nan;

O16pO18prof_AnM=sum(hand.O16pO18image_FiAl)./sum(hand.Mask_FiAl_Basic);
O18prof_AnM=sum(hand.O18image_FiAl)./sum(hand.Mask_FiAl_Basic);
NormO18prof_AnM=O18prof_AnM./O16pO18prof_AnM;
NormO18prof_AnM(isnan(NormO18prof_AnM))=nan;
NormO18prof_AnM(isinf(NormO18prof_AnM))=nan;

P(2,:)=NormO18prof_AnM;

%Profile based Masked not aligned
O16pO18prof_MnA=sum(hand.O16pO18image_CoAl.*hand.Mask_CoAl_Thresh);
O18prof_MnA=sum(hand.O18image_CoAl.*hand.Mask_CoAl_Thresh);
NormO18prof_MnA=O18prof_MnA./O16pO18prof_MnA;
NormO18prof_MnA(isnan(NormO18prof_MnA))=nan;
NormO18prof_MnA(isinf(NormO18prof_MnA))=nan;
P(3,:)=NormO18prof_MnA;

%Basic Profile Normailisation
O16pO18prof_Or=sum(hand.O16pO18image_CoAl.*hand.Mask_CoAl_Basic);
O18prof_Or=sum(hand.O18image_CoAl.*hand.Mask_CoAl_Basic);
NormO18prof_Or=O18prof_Or./O16pO18prof_Or;
NormO18prof_Or(isnan(NormO18prof_Or))=nan;
NormO18prof_Or(isinf(NormO18prof_Or))=nan;
NormO18prof_Or(NormO18prof_Or==1)=nan;
P(4,:)=NormO18prof_Or;

%Original image based normalisation
hand.NormO18prof_Or_image=hand.O18image_CoAl./hand.O16pO18image_CoAl;
hand.NormO18prof_Or_image(isnan(hand.NormO18prof_Or_image))=0;
P(5,:)=mean(hand.NormO18prof_Or_image);

hand.ProfileData_Or=NormO18prof_FiAl_MaTh;
[hand]=DataPlotOnly(hObject, eventdata, hand);
% set(hand.Ylimit,'Position',[26 525 40 20]);
%Setting evironment
ProfileSelection=get(hand.ProfileSelector, 'Value');
if ProfileSelection==6
    ProfileSelection=1;
end

% [SurfGrad SurfPos]=min(O16pO18prof_FiAl_MaTh(1:end-1)-O16pO18prof_FiAl_MaTh(2:end))
% [SurfGrad SurfPos]=min(O16pO18prof_AnM(1:end-1)-O16pO18prof_AnM(2:end))
%[SurfGrad SurfPos]=max(diff(O16pO18prof_AnM)); %Surface is at greatest gradient of rotated, but unmasked O16pO18image
% SurfPos=SurfPos+1

%Possibly start the fit where the surface roughness has stopped...
% set(hand.SurfPos,'String',...
%     num2str(roundsf(1e6*hand.Xdata_Or(SurfPos),2,'round')));

C_bg=median(roundsf(P(ProfileSelection,round(end/2):end),2,'round'));

% Pro_End=roundsf(max(1e6*hand.Xdata_Or)-...
%     str2double(get(hand.SurfPos,'String')),3,'floor');
% if C_bg<0.004 && C_bg>0.001
%     Fit_End=roundsf(1.2*(1e6*hand.Xdata_Or(find(...
%         roundsf(NormO18prof_FiAl_MaTh(hand.SurfPos_idx:end),2,'round' )<...
%         roundsf(C_bg,2,'round'),1,'first')...
%         )-str2double(get(hand.SurfPos,'String'))),3,'round');
%
%     %     find(NormO18prof_FiAl_MaTh
%
%     %     Fit_End=1.1*(num2str(roundsf(1e6*hand.Xdata_Or(find(...
%     %         roundsf(NormO18prof_FiAl_MaTh,2,'round' )<...
%     %         roundsf(C_bg,2,'round'),1,'first')),2,'round')-...
%     %         roundsf(str2double(get(hand.SurfPos,'String')),3,'floor')))
%     if Fit_End>Pro_End
%         Fit_End=Pro_End;
%     else
%     end
% else
%     C_bg=0.002;
%     Fit_End=Pro_End;
% end
hand.counts=sum(hand.O16pO18image_FiAl)./sum(hand.Mask_FiAl_Basic);
hand.counts(hand.counts==inf)=0;
% find(roundsf(hand.counts,2,'round')>0.99*mean(hand.counts),1,'first')
% [RatMax Fit_Start]=max(NormO18prof_FiAl_MaTh(SurfPos:end));
% set(hand.Fit_Start,'String',...
%     num2str(roundsf(1e6*hand.Xdata_Or(Fit_Start),2,'round')));
% pause(1)

Bmean_idx=find(roundsf(hand.counts,2,'round')>mean(hand.counts),1,'first');
ProStart_idx=find(P(ProfileSelection,:)>0,1);
if max(Bmean_idx,ProStart_idx)-hand.SurfPos_idx>0
    Fit_Start_idx=max(Bmean_idx,ProStart_idx)-hand.SurfPos_idx;
else
    Fit_Start_idx=1;
end
Fit_Start=1e6*hand.X(Fit_Start_idx);

% Bmean=1e6*hand.Xdata_Or(find(roundsf(hand.counts,2,'round')>...
%     mean(hand.counts),1,'first'));
% if Bmean<str2double(get(hand.SurfPos,'String'))
%     Fit_Start=0;
% else
%     Fit_Start=Bmean-str2double(get(hand.SurfPos,'String'));
% end
% % Fit_Start=(roundsf(1e6*hand.Xdata(find(...
% %     roundsf(hand.counts,2,'round')>0.99*mean(hand.counts)...
% %     ,1,'first')),2,'round'))-str2double(get(hand.SurfPos,'String'));
% Pro_Start=-roundsf(1e6*hand.Xdata_Or(find(P(ProfileSelection,:)>0,1))-...
%     str2double(get(hand.SurfPos,'String')),3,'floor')
% % find(hand.P(ProfileSelection,:)>0,1)
% % hand.SurfPos_idx
%
% if find(P(ProfileSelection,:)>0,1)>hand.SurfPos_idx
%     if 1e6*hand.X(find(P(ProfileSelection,:)>0,1)-hand.SurfPos_idx)>Fit_Start
%         Fit_Start=1e6*hand.X(find(P(ProfileSelection,:)>0,1)-hand.SurfPos_idx+1);
%     end
% end
%
% if find(P(ProfileSelection,:)>0,1,'last')>hand.SurfPos_idx
%     if 1e6*hand.X(find(P(ProfileSelection,:)>0,1,'last')-hand.SurfPos_idx)<Fit_End
%         Fit_End=1e6*hand.X(find(P(ProfileSelection,:)>0,1,'last')-hand.SurfPos_idx);
%     end
% end

% if hand.ROIflag==1
%     if hand.ROImin-str2double(get(hand.SurfPos,'String'))>Fit_Start
%         Fit_Start=hand.ROImin-str2double(get(hand.SurfPos,'String'))+2;
%     end
%     if hand.ROImax-str2double(get(hand.SurfPos,'String'))<Fit_End
%         Fit_End=hand.ROImax-str2double(get(hand.SurfPos,'String'))-2;
%     end
% % end
% if Fit_Start<Pro_Start
%     Fit_Start=Pro_Start;
% end
% if ~isfinite(C_bg)
%     C_bg=0.002;
% end
% set(hand.C_bg,'String',C_bg);
% set(hand.C_bg,'BackgroundColor',[0,1,0]);
% if Fit_End<0 || ~isfinite(Fit_End)
%     Fit_End=0.5*str2double(get(hand.ProfileLength,'String'));
% else
%     set(hand.Fit_End,'BackgroundColor',[0,1,0]);
% end
% set(hand.Fit_End,'BackgroundColor',[0,1,0]);
if Fit_Start<0
    Fit_Start=0;
elseif Fit_Start>str2double(get(hand.ProfileLength, 'String'))/3;
    Fit_Start=0;
else
    set(hand.Fit_Start,'BackgroundColor',[0,1,0]);
end
set(hand.Fit_Start,'String',roundsf(Fit_Start,3,'ceil'));

hand.P=P;
%Plotting
[hand]=ProfileSelector_Callback(hObject, eventdata, hand);
guidata(hObject, hand);
function GenerateProfiles_CreateFcn(hObject, eventdata, hand)

function [hand]=ProfileSelector_Callback(hObject, eventdata, hand)
%Plotting
hand.CurrentPlot='Generate';
hand.ProfileGenFlag=1;
ProfileLength=str2double(get(hand.ProfileLength, 'String'));
SurfPos=str2double(get(hand.SurfPos, 'String'));
PixelNo=str2double(get(hand.PixelNo, 'String'));
% DataStep=ProfileLength*1e-6/PixelNo;
% [m,n]=size(hand.ProfileData_Or);
% hand.Xdata_Or=0:DataStep:(max(m,n)-1)*DataStep;
dx=ProfileLength*1e-6/(PixelNo-1); %spatial step
X=0:dx:ProfileLength*1e-6; %domain
hand.Xdata_Or=X;
pX=hand.Xdata_Or*1e6;
P=hand.P;

%NormO18image_FiAl_MaTh=hand.O16pO18image.Aligned./(hand.O16image+hand.O18image);
if get(hand.ProfileSelector, 'Value')==6
    hand.ProfileData_Or=P(1,:);
    plot(pX,P(1,:),pX,P(2,:),pX,P(3,:),pX,P(4,:),pX,P(5,:));
    axis square;
    legend('Aligned and Masked','Aligned Only','Masked Only','Basic Norm. Prof.','Original Norm. Image')
    set(hand.Ylimit,'String',ceil(max(hand.ProfileData_Or)*110)/100);
    
    %     Ylimit_Callback(hObject, eventdata, hand);
    set(hand.Ylimit,'Visible','on');
    set(hand.Ylimit,'Position',hand.ImagePlotPosYlim);
    set(hand.Xlimit,'Position',hand.ImagePlotPosXlim);
    %     hand.SaveName=[hand.SaveName_Or(1:end-4),'(all).txt'];
    set(hand.ProfDataSaveName, 'String',[hand.SaveName_Or(1:end-4),'(all).txt']);
    % set(gca,'position',[90    60   465   465]);%pix
    set(gca,'position',hand.ImagePlotPos); %norm
    axis square;
else
    set(hand.Ylimit,'Visible','on');
    switch get(hand.ProfileSelector, 'Value')
        case 1;
            hand.ProfileData_Or=P(1,:);
            %             hand.SaveName=[hand.SaveName_Or(1:end-4),'(A_M).txt'];
            set(hand.ProfDataSaveName, 'String',[hand.SaveName_Or(1:end-4),'(A_M).txt']);
        case 2;
            hand.ProfileData_Or=P(2,:);
            %             hand.SaveName=[hand.SaveName_Or(1:end-4),'(A).txt'];
            set(hand.ProfDataSaveName, 'String',[hand.SaveName_Or(1:end-4),'(A).txt']);
        case 3;
            hand.ProfileData_Or=P(3,:);
            %             hand.SaveName=[hand.SaveName_Or(1:end-4),'(M).txt'];
            set(hand.ProfDataSaveName, 'String',[hand.SaveName_Or(1:end-4),'(M).txt']);
        case 4;
            hand.ProfileData_Or=P(4,:);
            %             hand.SaveName=[hand.SaveName_Or(1:end-4),'(NormProf).txt'];
            set(hand.ProfDataSaveName, 'String',[hand.SaveName_Or(1:end-4),'(NormProf).txt']);
        case 5;
            hand.ProfileData_Or=P(5,:);
            %             hand.SaveName=[hand.SaveName_Or(1:end-4),'(Or).txt'];
            set(hand.ProfDataSaveName, 'String',[hand.SaveName_Or(1:end-4),'(Or).txt']);
    end
    %     set(hand.Ylimit,'String',roundsf(max(hand.ProfileData_Or)*1.1,2,'ceil'));
    set(hand.Ylimit,'String',ceil(max(hand.ProfileData_Or(X>SurfPos*1e-6))*105)/100);
    %     set(hand.Ylimit,'String',1);
    imagesc([0 str2num(get(hand.ProfileLength,'String'))],[0 str2num(get(hand.Ylimit,'String'))],...
        flipud(hand.NormO18image_FiAl_MaTh)); hold on; axis square;
    hand.colo=colorbar; ylabel(hand.colo,'Isotopic Fraction');
    set(hand.colo,'FontSize',10);
    set(hand.Ylimit,'Position',hand.ImagePlotPosYlim);
    set(hand.Xlimit,'Position',hand.ImagePlotPosXlim);
    % set(gca,'position',[90    60   465   465]);%pix
    set(gca,'position',hand.ImagePlotPos); %norm
    %     if length(get(hand.CprimeY,'State'))==2;
    %         C_bg=str2double(get(hand.C_bg, 'String'));
    %         C_gas=str2double(get(hand.C_gas, 'String'));
    %         pro=(hand.ProfileData_Or-C_bg)/(C_gas-C_bg);
    %         ylabel('Normalised _{}^{18}O Fraction, C''');
    %     else
    pro=hand.ProfileData_Or;
    %         ylabel('Isotopic Fraction');
    %     end
    hAx=plot(pX,pro,'w','linewidth',2.5);
    %     plot(pX,,pX,hand.ProfileData_Or,'w','linewidth',2.8);
    set(gca,'ydir','normal');
    %%%Show surface position
    
    %     hand.SurfLine=imline(gca,[SurfPos, SurfPos], [-5000, 5000]);%,'Color','w','LineStyle','--');
    %     hand.h1 = get(hand.SurfLine,'Children');
    %     set(hand.h1,'LineStyle',':');
    %     set(hand.h1,'LineWidth',2)
    %     setColor(hand.SurfLine,[0.1 1 0.1]);
    hand.SurfLine2=line([SurfPos, SurfPos],[-5000 5000],...
        'LineWidth',2,'Color',[0 1 1],'LineStyle',':');
    %%Show Cbg level
    C_bg=str2double(get(hand.C_bg,'String'));
    hand.C_bgLine=line([-5000 5000],[C_bg, C_bg],...
        'LineWidth',2,'Color',[.1 .1 1],'LineStyle',':');
    %     hand.C_bgLine=imline(gca, [-5000, 5000],[C_bg, C_bg]);%,'Color','w','LineStyle','--');
    %     h2 = get(hand.C_bgLine,'Children');
    %     set(h2,'LineStyle',':');
    %     set(h2,'LineWidth',2)
    %     legend(hAx,'Profile',hand.SurfLine,'Surface position',hand.C_bg,'Background concentration')
    %     setColor(hand.SurfLine,[0.1 1 0.1]);
end
hand.Ylimit_min=0;
Ylimit_Callback(hObject, eventdata, hand);
xlabel('Depth /\mum');ylabel('Isotopic Fraction');
Xlimit_Callback(hObject, eventdata, hand);
hold off;
grid (gca,get(hand.GridLines,'State'))
set(gca,'Xcolor',[0.3 0.3 0.3]); set(gca,'Ycolor',[0.3 0.3 0.3]);
% hand.SaveName=[hand.FileName(1:end-4),...
%     '_Norm18Prof(',num2str(get(hand.ProfileSelector, 'Value')),').txt'];
% hand.SaveName=[hand.PathName,'\',hand.FileName(1:end-4),...
%     '_Norm18Prof(',num2str(get(hand.ProfileSelector, 'Value')),').txt'];
PlotProperties(hObject, eventdata, hand)
guidata(hObject, hand);
function ProfileSelector_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ProfDataSave_Callback(hObject, eventdata, hand)
%Saving profiles
% hand.Name=get(hand.ProfDataSaveName,'String');
SaveName=get(hand.ProfDataSaveName,'String');
[FileName,PathName]=uiputfile([hand.PathName,SaveName]);
fid=fopen([FileName,PathName],'wt');
if get(hand.ProfileSelector, 'Value')==6;
    fprintf(fid,'%d, %d, %d, %d \n',hand.P);
else
    fprintf(fid,'%d\n',hand.P(get(hand.ProfileSelector, 'Value'),:));
end
fclose(fid);
guidata(hObject, hand);


function ProfDataSaveName_Callback(hObject, eventdata, hand)
guidata(hObject, hand);
function ProfDataSaveName_CreateFcn(hObject, eventdata, hand)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function [y]=roundsf(number,sfs,method)
%opt = {'round','floor','ceil','fix'};
og = 10.^(floor(log10(abs(number)) - sfs + 1));
y = feval(method,number./og).*og;
y(find(number==0)) = 0;

function [r2 rmse] = rsquare(y,f,varargin)
if isempty(varargin); c = true;
elseif length(varargin)>1; error 'Too many input arguments';
elseif ~islogical(varargin{1}); error 'C must be logical (TRUE||FALSE)'
else c = varargin{1};
end

% Compare inputs
if ~all(size(y)==size(f)); error 'Y and F must be the same size'; end

% Check for NaN
tmp = ~or(isnan(y),isnan(f));
y = y(tmp);
f = f(tmp);

if c; r2 = max(0,1 - sum((y(:)-f(:)).^2)/sum((y(:)-mean(y(:))).^2,'omitnan'));
else r2 = 1 - sum((y(:)-f(:)).^2)/sum((y(:)).^2);
    if r2<0
        % http://web.maths.unsw.edu.au/~adelle/Garvan/Assays/GoodnessOfFit.html
        warning('Consider adding a constant term to your model') %#ok<WNTAG>
        r2 = 0;
    end
end

rmse = sqrt(mean((y(:) - f(:)).^2));

function [pro]=BackDiffs_AutoCaller(p,b,ran,FitCheck,BC,MP_m)
if min(p)<=0
    pro=zeros(1,ran(2)-ran(1)+1);
else
    % Check which parameters need to be fitted/fixed
    if sum(FitCheck)==4
        D1=p(1);        D2=p(2);        k1=p(3);        k2=p(4);
    elseif FitCheck(2)==0
        D1=p(1);        D2=p(1);        k1=p(2);        k2=p(3);
    elseif FitCheck(4)==0
        D1=p(1);        D2=p(2);        k1=p(3);        k2=p(3);
    end
    
    %%%%%%%%%%%%
    % p(1:2)
    % p(3:4)
    %%%%%%%%%%%%
    C_gas=b(1);
    C_bg=b(2);
    t1=b(3);
    t2=b(4);
    ProLen_m=b(5);
    PixelNo=b(6);
    
    [X,pro]=CN_inline(...
        C_gas,C_bg,D1,D2,k1,k2,t1,t2,ProLen_m,PixelNo,BC,MP_m);
    pro=pro(ran(1):ran(2));
end


function WarningBox_CreateFcn(hObject, eventdata, hand)

function [X,pro]=Crank_SI_inline(C_gas,C_bg,D1,k1,t1,ProLen_m,PixelNo)
dx=ProLen_m/(PixelNo-1); %spatial step
X=0:dx:ProLen_m; %domain
h1=k1/D1;
t_tot=t1 ;
if D1>0 && k1>0
    C_prime=erfc(X./(2*sqrt(D1*t_tot)))-...
        exp(h1.*X+D1*t_tot*h1^2).*...
        erfc(X./(2*sqrt(D1*t_tot))+h1*sqrt(D1*t_tot));
else
    C_prime=ones(PixelNo);
end
pro=C_prime*(C_gas-C_bg)+C_bg;

function [X,pro]=Crank_PS_inline(C_gas,C_bg,D1,k1,t1,ProLen_m,PixelNo,BC,MP_m)
dx=ProLen_m/(PixelNo-1); %spatial step
X=0:dx:ProLen_m; %domain
[~, MP_idx] = min(abs(X-MP_m));
h1=k1/D1;
t_tot=t1 ;
x_ProEnd=4*2*sqrt(D1*t_tot);
if D1>0 && k1>0
    if (MP_m>x_ProEnd && MP_m>ProLen_m) || MP_m==0 % if the mirror is miles away then just use crank
        C_prime=erfc(X./(2*sqrt(D1*t_tot)))-...
            exp(h1.*X+D1*t_tot*h1^2).*...
            erfc(X./(2*sqrt(D1*t_tot))+h1*sqrt(D1*t_tot));
    else
        if D1>0 && k1>0
            [RootSum]=RootFinder(MP_m,k1,D1,X,t_tot);
            C_prime=1-sum(RootSum,1);
        else
            C_prime=ones(PixelNo);
        end
    end
else
    C_prime=ones(PixelNo);
end
pro=C_prime*(C_gas-C_bg)+C_bg;
if length(pro)>2*MP_idx
    pro(2*MP_idx:end)=nan;
end
if BC==2
    pro(MP_idx:end)=nan;
end

%% Function to find the roots in the analytical plane-sheet solution
function [RootSum]=RootFinder(MP_m,k1,D1,X,t_tot)
L=MP_m*k1/D1;
RootNo=round(3*(L+10)^.75);
beta=zeros(1,RootNo);
fun1 = @(beta) (beta*tan(beta)-L)^2;
for k=1:RootNo
    beta(k)=fminbnd(fun1,pi*(k-1),pi*(k-0.5));
end
for i=1:length(beta)
    RootSum(i,:)=2*L*cos(beta(i)*(X-MP_m)/MP_m)*...
        exp(-beta(i)^2*D1*t_tot/MP_m^2)/((beta(i)^2+L^2+L)*cos(beta(i)));
end

function [X,pro]=BackX_SI_inline(C_gas,C_bg,D1,k1,t1,t2,ProLen_m,PixelNo)
dx=ProLen_m/(PixelNo-1); %spatial step
X=0:dx:ProLen_m; %domain
h1=k1/D1;
t_tot=(t1+t2) ;
phi=t2/(t1+t2);
if D1>0 && k1>0
    if t2>0
        C_prime=...
            (erfc(X./(2*sqrt(D1*t_tot)))-exp(h1.*X+D1*t_tot*h1^2).*erfc(...
            X./(2*sqrt(D1*t_tot))+h1*sqrt(D1*t_tot))) - ...
            (erfc(X./(2*sqrt(D1*t_tot*phi)))-exp(h1.*X+D1*t_tot*phi*h1^2).*erfc(...
            X./(2*sqrt(D1*t_tot*phi))+h1*sqrt(D1*t_tot*phi)));
    else
        C_prime=erfc(X./(2*sqrt(D1*t_tot)))-...
            exp(h1.*X+D1*t_tot*h1^2).*...
            erfc(X./(2*sqrt(D1*t_tot))+h1*sqrt(D1*t_tot));
    end
else
    C_prime=ones(PixelNo);
end
pro=C_prime*(C_gas-C_bg)+C_bg;

function [X,pro]=CN_inline(C_gas,C_bg,D1,D2,k1,k2,t1,t2,ProLen_m,PixelNo,BC,MP_m)
t_i=t1 ; %s %initial exchange duration
t_b=t2 ; %s %back exchange duration
t_tot=t_i+t_b; %s %total time
h1=k1/D1;
h2=k2/D2;
C_gas1=C_gas; % gas concentration for period 1
C_gas2=C_bg; % gas concentration for period 2
PixelNo_Data=PixelNo; %Store the orinigal number of data point for interpolation at the end
PixelNo=300; %Run the simulation on a sensible number of points and then interpolate to the true positions afterwards

%% Generate analytical exchange 1 where possible
if BC==1
    dx=ProLen_m/(PixelNo-1); %spatial step
    X=0:dx:ProLen_m; %domain
    C_X1=(C_bg+(C_gas-C_bg)*...
        (erfc(X./(2*sqrt(D1*t_i)))-...
        exp(h1.*X+t_i*D1*h1^2)...
        .*erfc(X./(2*sqrt(D1*t_i))+h1*sqrt(D1*t_i))))';
else
    dx=MP_m/(PixelNo-1); %spatial step
    X=0:dx:MP_m; %domain
    [~,C_X1]=Crank_PS_inline(C_gas,C_bg,D1,k1,t1,MP_m,PixelNo,BC,MP_m);
    C_X1=C_X1';
end
dt=5*round(dx^2/(2*min(D1,D2)),3,'significant'); %time step
Nt=round((t_tot)/dt); % Number of time steps

%% Exchanges
if min([D1,D2,k1,k2])<0
    pro=2*ones(PixelNo,1); %Give a flat line if user or optimser is silly
else
    C=C_bg*ones(PixelNo,1);
    I=eye(PixelNo);
    
    %% Exchange 1
    if ~isfinite(sum(C_X1)) % If the analytical approach fails, simulate it
        %%% Step 1
        sigma1=D1*dt/(2*dx^2);
        %%% CN Diffusion matrix
        A1=    full(gallery('tridiag',PixelNo, sigma1,1-2*sigma1, sigma1));
        A1_new=full(gallery('tridiag',PixelNo,-sigma1,1+2*sigma1,-sigma1));
        %%% Exchange surface condition
        A1(1,1:2)=    [1-2*dx*h1*sigma1-2*sigma1, 2*sigma1];
        A1_new(1,1:2)=[1+2*dx*h1*sigma1+2*sigma1,-2*sigma1];
        beta1=-h1*C_gas1;
        G1=zeros(PixelNo,1); G1(1)=-4*dx*beta1*sigma1;
        %%% Mirror Boundary condition
        A1(end,end-2:end)=    [sigma1 -2*sigma1 1+sigma1];
        A1_new(end,end-2:end)=[-sigma1 2*sigma1 1-sigma1];
        A1_newI=A1_new\I;
        AA1=A1_newI*A1;
        AG1=A1_newI*G1;
        for t_idx=1:ceil(t_i/dt) %%%% double  check this
            C=AA1*C+AG1;
        end
        C1=C;
    else
        C1=C_X1;
        t_idx=ceil(t_i/dt);
    end
    C=C1;
    
    %% Exchange 2
    sigma2=D2*dt/(2*dx^2);
    %%% CN Diffusion matrix
    A2=    full(gallery('tridiag',PixelNo, sigma2,1-2*sigma2, sigma2));
    A2_new=full(gallery('tridiag',PixelNo,-sigma2,1+2*sigma2,-sigma2));
    %%% Exchange surface condition
    A2(1,1:2)=    [1-2*dx*h2*sigma2-2*sigma2, 2*sigma2];
    A2_new(1,1:2)=[1+2*dx*h2*sigma2+2*sigma2,-2*sigma2];
    beta2=-h2*C_gas2;
    G2=zeros(PixelNo,1); G2(1)=-4*dx*beta2*sigma2;
    %%% Mirror Boundary condition
    A2(end,end-2:end)=    [sigma2 -2*sigma2 1+sigma2];
    A2_new(end,end-2:end)=[-sigma2 2*sigma2 1-sigma2];
    A2_newI=A2_new\I;
    %
    AA2=A2_newI*A2;
    AG2=A2_newI*G2;
    for t_idx=1:Nt-t_idx+1   %% double check
        C=AA2*C+AG2;
    end
    
    %% Boundary adjustments
    if BC~=1 %% Apply mirror to double length of plane-sheet
        pro=[C(1:end-1);flipud(C)];
        X=0:dx:2*MP_m;
    else
        %Approximate semi-infinite by taking average of mirror and C(x=L)=0
        if C(end)<C_bg+(C_gas1-C_bg)*0.1 % if this end is low after both excahnges, then don't bother
            pro=C;
        else
            C_mirror=C; % Store the result of the first simulation w/ mirror
            %% Simulate again
            C=C_X1;
            
            if C(end)>C_bg+(C_gas1-C_bg)*0.1 % if this end is low after exchange 1, don't bother
                %%% Linear grad.
                A1(end,end-2:end)=    [-sigma1 2*sigma1 1-sigma1]; %central dif
                A1_new(end,end-2:end)=[sigma1 -2*sigma1 1+sigma1];
                A1_newI=A1_new\I;
                AA1=A1_newI*A1;
                AG1=A1_newI*G1;
                for t_idx=1:ceil(t_i/dt)
                    C1=AA1*C+AG1;
                end
            else
                C1=C_X1;
                t_idx=ceil(t_i/dt);
            end
            
            C=C1;            
            A2(end,end-2:end)=    [-sigma2 2*sigma2 1-sigma2];
            A2_new(end,end-2:end)=[sigma2 -2*sigma2 1+sigma2];
            A2_newI=A2_new\I;
            % Time stepping
            AA2=A2_newI*A2;
            AG2=A2_newI*G2;
            for t_idx=1:Nt-t_idx+1   %% double check
                C=AA2*C+AG2;
            end
            pro=(C_mirror+C)./2;
        end
    end 
end

PixelNo=PixelNo_Data;
dx=ProLen_m/(PixelNo-1); %spatial step
X_sim=X;
X=0:dx:ProLen_m; %domain
pro=interp1(X_sim,pro,X,'spline');
if max(X)>max(X_sim)
    pro(X>max(X_sim))=nan;
end
if BC==2
    pro(X>MP_m)=nan;
end

function [X,pro]=CN_Interface_inline(C_gas,C_bg,D1,D2,k1,r_int,t1,L_int_m,ProLen_m,PixelNo,BC,MP_m)
t_i=t1 ; %s %initial exchange duration
h1=k1/D1;
%PixelNo_Data=PixelNo; %Store the orinigal number of data point for interpolation at the end
%PixelNo=300; %Perhaps run the simulation on a sensible number of points and then interpolate to the true positions afterwards

dx=ProLen_m/(PixelNo-1);%spatial step
X=0:dx:ProLen_m; %domain
t=t_i; %s %initial exchange duration
%dt=5*round(dx^2/(2*min(D1,D2)),3,'significant'); % Time step
dt = 1;
Nt=t/dt; % Number of time steps round((t_tot)/dt)
Nx=ProLen_m/dx+1; % Define number of spatial nodes

L = [L_int_m;ProLen_m-L_int_m]; % Vector of the layer lengths, first element is the first layer
D_int=2/(2*r_int/dx+1/D1+1/D2);

sigma_1=D1*(dt)/(2*dx^2); % Calculate sigma of LSCF
sigma_2=D2*(dt)/(2*dx^2); % Calculate sigma of GDC
sigma_int=D_int*(dt)/(2*dx^2);% Calculate sigma of the first interface

%%

% Time stepping
%% Build simulation components
% Set up a sub-matrix L which contains the information regarding the surface exchange as well as diffusion through the LSCF layer

% Set up the vector for the subdiagonal
sub = zeros(round(Nx)-1,1);
sub(1:round(L(1)/dx)-1,1)=sigma_1;
sub(round(L(1)/dx):round(L(1)/dx)+1,1)=sigma_int;
sub(round(L(1)/dx)+1:end,1)=sigma_2;

% Set up the vector for the superdiagonal
sup = zeros(round(Nx)-1,1);
sup(1:round(L(1)/dx)-1,1)=sigma_1;
sup(round(L(1)/dx)-0:round(L(1)/dx),1)=sigma_int;
sup(round(L(1)/dx)+1:end,1)=sigma_2;
% Set up the vector for the central diagonal
s1 = zeros(round(Nx),1);
s1(1:round(L(1)/dx)-1,1)=1-2*sigma_1;
s1(round(L(1)/dx),1)=1-(sigma_int+sigma_1);
s1(round(L(1)/dx)+1,1)=1-(sigma_int+sigma_2);
s1(round(L(1)/dx)+2:end,1)=1-2*sigma_2;

s1_n = zeros(round(Nx),1);
s1_n(1:round(L(1)/dx)-1,1)=1+2*sigma_1;
s1_n(round(L(1)/dx),1)=1+(sigma_int+sigma_1);
s1_n(round(L(1)/dx)+1,1)=1+(sigma_int+sigma_2);
s1_n(round(L(1)/dx)+2:end,1)=1+2*sigma_2;

% Create the matrix at the current timestep (Mirror boundary condition)
A = full(gallery('tridiag',sub,s1,sup));
A(1,1:2) = [1-2*sigma_1*(1+dx*h1) 2*sigma_1]; %Surface boundary condition
A(end,end-2:end) = [sigma_2 -2*sigma_2 1+sigma_2]; %Mirror boundary condition parameters

% Create the matrix at the future timestep
A_n = full(gallery('tridiag',-sub,s1_n,-sup));
A_n(1,1:2) = [1+2*sigma_1*(1+dx*h1) -2*sigma_1]; %Surface boundary condition
A_n(end,end-2:end) = [-sigma_2 2*sigma_2 1-sigma_2]; %Mirror boundary condition parameters

% Create the matrix at the current timestep (Dirichlet boundary condition)
A_Di = full(gallery('tridiag',sub,s1,sup));
A_Di(1,1:2) = [1-2*sigma_1*(1+dx*h1), 2*sigma_1]; %Surface boundary condition
A_Di(end,end-2:end) = [0 0 1]; % Dirichlet boundary condition

A_n_Di = full(gallery('tridiag',-sub,-s1+2,-sup));
A_n_Di(1,1:2) = [1+2*sigma_1*(1+dx*h1), -2*sigma_1]; %Surface boundary condition
A_n_Di(end,end-2:end) = [0 0 1]; %Mirror boundary condition parameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define surface exchange vector
G = zeros(round(Nx), 1);
G(1) = 4*dx*sigma_1*h1*C_gas; %surface boundary condition vector G
C= C_bg*ones(round(Nx),1); %Initial position vector, 0 everywhere
C_Di = C_bg*ones(round(Nx),1);% Initial position vector for the Dirichlet bc


%% Run the iterative matrix calculation

A_nI = inv(A_n);
A_nI_Di = inv(A_n_Di);

for t=1:Nt
    C = A_nI*(A*C+G);
end

if C(end) < 0.001
    pro = C;
else
    
    for t=1:Nt
        C_Di = A_nI_Di*(A_Di*C_Di+G);
    end

    
    pro = (C+C_Di)/2;
    %pro=interp1(X_sim,pro,X,'spline');
end

function [D1,k1]= AutoFit_Crank(hand)
C_bg=str2double(get(hand.C_bg, 'String'));
C_gas=str2double(get(hand.C_gas, 'String'));
D1=str2double(get(hand.D1, 'String'));
k1=str2double(get(hand.k1, 'String'));
t1=3600*str2double(get(hand.t1, 'String'));
X_temp = hand.Xdata(hand.Fit_Start_idx:hand.Fit_End_idx);
t_tot=t1 ;
fun = @(p) sum((hand.ProfileData(hand.Fit_Start_idx:hand.Fit_End_idx) - ...
    (C_bg+(C_gas-C_bg)*...
    (erfc(X_temp./(2*sqrt(abs(p(1))*t_tot)))-...
    exp(abs(p(2)).*X_temp+abs(p(1))*t_tot*abs(p(2))^2)...
    .*erfc(...
    X_temp./(2*sqrt(abs(p(1))*t_tot))...
    +abs(p(2))*sqrt(abs(p(1))*t_tot)))...
    )).^2,'omitnan');
h1=abs(k1/D1); %5.5e3
%starting guess
pguess = [abs(D1),h1];
%optimise
[p,~] = fminsearch(fun,pguess);
% [p,fminres] = fminsearch(fun,pguess,optimset('Display','iter'));
D1=abs(p(1));
k1=abs(p(1)*p(2));

function [D1,k1,A_gb,Z_gb]=AutoFit_Crank_LC(hand)
C_bg=str2double(get(hand.C_bg, 'String'));
C_gas=str2double(get(hand.C_gas, 'String'));
D1=str2double(get(hand.D1, 'String'));
k1=str2double(get(hand.k1, 'String'));
t1=3600*str2double(get(hand.t1, 'String'));
X_temp = hand.Xdata(hand.Fit_Start_idx:hand.Fit_End_idx);
t_tot=t1 ;
fun = @(p) sum((hand.ProfileData(hand.Fit_Start_idx:hand.Fit_End_idx) - ...
    (C_bg+(C_gas-C_bg)*...
    (erfc(X_temp./(2*sqrt(abs(p(1))*t_tot)))-...
    exp(abs(p(2)).*X_temp+abs(p(1))*t_tot*abs(p(2))^2)...
    .*erfc(...
    X_temp./(2*sqrt(abs(p(1))*t_tot))...
    +abs(p(2))*sqrt(abs(p(1))*t_tot)))+...
    abs(p(3))*exp(-abs(p(4))*X_temp.^(6/5))...
    )).^2,'omitnan');
h1=abs(k1/D1); %5.5e3
%starting guess
pguess = [abs(D1),h1,1e-9,1e-9];
%optimise
[p,fminres] = fminsearch(fun,pguess);
% [p,fminres] = fminsearch(fun,pguess,optimset('Display','iter'));
D1=abs(p(1));
k1=abs(p(1)*p(2));
A_gb=abs(p(3));
Z_gb=abs(p(4));
%
% Plotting
hand.newFig=figure;
set(hand.newFig,'Color',[1 1 1]);
set(hand.newFig,'WindowStyle','normal')
set(hand.newFig,'PaperPositionMode','auto');
set(hand.newFig,'PaperOrientation','landscape');
set(hand.newFig,'Position',[60 50 1200 800]);
set(hand.newFig,'Position',[60 50 1200 800]);
set(hand.newFig,'renderer','painters')

Crank_LC=C_bg+(C_gas-C_bg)*...
    (erfc(hand.Xdata./(2*sqrt(abs(p(1))*t_tot)))-...
    exp(abs(p(2)).*hand.Xdata+abs(p(1))*t_tot*abs(p(2))^2)...
    .*erfc(...
    hand.Xdata./(2*sqrt(abs(p(1))*t_tot))...
    +abs(p(2))*sqrt(abs(p(1))*t_tot)))+...
    p(3)*exp(-p(4)*hand.Xdata.^(6/5));

Crank=C_bg+(C_gas-C_bg)*...
    (erfc(hand.Xdata./(2*sqrt(abs(p(1))*t_tot)))-...
    exp(abs(p(2)).*hand.Xdata+abs(p(1))*t_tot*abs(p(2))^2)...
    .*erfc(...
    hand.Xdata./(2*sqrt(abs(p(1))*t_tot))...
    +abs(p(2))*sqrt(abs(p(1))*t_tot)));


if length(get(hand.GBplot,'State'))==3
    if length(get(hand.CprimeY,'State'))==3
        ylabel('Isotopic Fraction, C');
        ProfileData=hand.ProfileData;
    else
        Crank_LC=((Crank_LC-C_bg)/(C_gas-C_bg));
        Crank=(Crank-C_bg)/(C_gas-C_bg);
        ProfileData=((hand.ProfileData-C_bg)/(C_gas-C_bg));
        ylabel('Normalised Isotopic Fraction, $C''$');
    end
    if length(get(hand.XprimeTog,'State'))==2
        X=hand.Xdata/(2*sqrt(D1*(t1) ));
        xlabel('Normalised depth, $x''$');
        set(gca,'xtick',[0:0.5:3]);
    else
        X=hand.Xdata*1e6;
        xlabel('Depth, $x$ /$\mu$m');
    end
    [hline]=plot(X,ProfileData,'o',X,Crank_LC,X,Crank);
    if length(get(hand.CprimeY,'State'))==3
        ylabel('Isotopic Fraction, C');
    else
        ylabel('Normalised Isotopic Fraction, $C''$');
    end
    if length(get(hand.XprimeTog,'State'))==2
        xlabel('Normalised depth, $x''$');
        set(gca,'xtick',[0:0.5:3]);
    else
        xlabel('Depth, $x$ /$\mu$m');
    end
else
    X=((hand.Xdata./ ((D1*t1 )^0.5) ).^(6/5));
    [hline]=plot(X,log((hand.ProfileData-C_bg)/(C_gas-C_bg)),'o',X,log((Crank_LC-C_bg)/(C_gas-C_bg)),X,log((Crank-C_bg)/(C_gas-C_bg)));
    ylabel('Log. Normalised Isotopic Fraction, ln$|C''|$');
    xlabel('Normalised Depth, $\eta^{6/5}$');
    set(gca,'xtick',[0:2:12]);
    ylim([-20 0]);
    xlim([0 12]);
end
set(get(gca,'xlabel'),'Interpreter','Latex');
set(get(gca,'ylabel'),'Interpreter','Latex');

set(gca,'position',[0.14 0.12 0.8 0.82])
set(gca,'Parent',hand.newFig,...
    'LineWidth',1.5,...
    'FontSize',24,...
    'TickLabelInterpreter','latex');
set(hline,'MarkerSize',4)
set(hline,'LineWidth',1.5)
legend('Data','Le Claire','Crank')
set(legend,'Interpreter','latex');
set(legend,'Position',[0.75 0.82 0.10 0.03])
set(legend,'Box','off')
figure(hand.MainWindow)
%
D_gb_delta=1.32*sqrt(D1/t_tot)*p(4)^(-5/3);
beta=D_gb_delta/(2*D1^(3/2)*t_tot^0.5);
Results=['\n ======RESULTS======\n',...
    '\nD = ',num2str(roundsf(p(1),3,'round')),' m^2/s',...
    '\nk = ',num2str(roundsf(k1,3,'round')),' m/s',...
    '\nA_gb = ',num2str(roundsf(abs(p(3)),3,'round')),...
    '\nZ_gb = ',num2str(roundsf(abs(p(4)),3,'round')),...
    '\nD_gb*delta = ',num2str(roundsf(D_gb_delta,3,'round')),...
    '\nbeta = ',num2str(roundsf(beta,3,'round')),...
    ];
disp(sprintf(Results))

function [D1,k1]=AutoFit_PlaneSheet(hand)
D1=str2double(get(hand.D1, 'String'));
k1=str2double(get(hand.k1, 'String'));
fun = @(p) sum((...
    hand.ProfileData(hand.Fit_Start_idx:hand.Fit_End_idx)-...
    PlaneSheet_AutoCaller(p,hand)).^2,'omitnan');
%starting guess
pguess = [abs(D1),k1];
%optimise
[p,fminres] = fminsearch(fun,pguess);
D1=abs(p(1));
k1=abs(p(2));

function [pro]=PlaneSheet_AutoCaller(p,hand)
set(hand.WarningBox,'Visible','off')
BC=get(hand.BoundCondMode,'Value');
C_bg=str2double(get(hand.C_bg, 'String'));
C_gas=str2double(get(hand.C_gas, 'String'));
ProLen_m=1e-6*str2double(get(hand.ProfileLength, 'String'));
PixelNo=str2double(get(hand.PixelNo, 'String'));
dx=ProLen_m/(PixelNo-1); %spatial step
X=0:dx:ProLen_m; %domain

D1=p(1);
k1=p(2);
t1=3600*str2double(get(hand.t1, 'String'));
t_tot=t1;
MP_m=str2double(get(hand.MirrorPlane,'String'))*1e-6;
[~, MP_idx] = min(abs(X-MP_m));
if MP_m<10*ProLen_m
    if D1<0 || k1<0
        C_prime=ones(PixelNo);
    else
        [RootSum]=RootFinder(MP_m,k1,D1,X,t_tot);
        C_prime=1-sum(RootSum,1);
    end
    pro=C_prime(hand.Fit_Start_idx:hand.Fit_End_idx)*(C_gas-C_bg)+C_bg;
    if length(pro)>2*MP_idx
        pro(2*MP_idx:end)=0;
    end
else
    [~,pro]=Crank_SI_inline(C_gas,C_bg,D1,k1,t1,ProLen_m,PixelNo,hand);
end
if BC==2
    pro(MP_idx:end)=nan;
end

function [D1,k1]= AutoFit_BackCrank(C_gas,C_bg,D1,k1,t1,t2,ProfileData,...
    Xdata,Fit_Start_idx,Fit_End_idx)
X_temp = Xdata(Fit_Start_idx:Fit_End_idx);
t_tot=(t1+t2) ;
phi=t2/(t1+t2);
% abs values used to ensure sqrt(p) is real
fun = @(p) sum((...
    ProfileData(Fit_Start_idx:Fit_End_idx)-...
    (C_bg+(C_gas-C_bg)*(...
    (erfc(X_temp./(2*sqrt(abs(p(1)*t_tot))))-exp(abs(p(2)).*X_temp+abs(p(1))*t_tot*abs(p(2))^2).*erfc(...
    X_temp./(2*sqrt(abs(p(1)*t_tot)))+abs(p(2))*sqrt(abs(abs(p(1))*t_tot)))) - ...
    (erfc(X_temp./(2*sqrt(abs(p(1)*t_tot*phi))))-exp(abs(p(2)).*X_temp+abs(p(1))*t_tot*phi*abs(p(2))^2).*erfc(...
    X_temp./(2*sqrt(abs(p(1)*t_tot*phi)))+abs(p(2))*sqrt(abs(p(1)*t_tot*phi))))...
    ))...
    ).^2,'omitnan');
h1=abs(k1/D1); %5.5e3
%starting guess
pguess = [abs(D1),h1];
%optimise
[p,~] = fminsearch(fun,pguess);
D1=p(1);
k1=p(1)*p(2);

function [D1,k1,D2,k2]= AutoFit_BackDiffs(...
    C_gas,C_bg,D1,k1,D2,k2,t1,t2,ProfileData,...
    Fit_Start_idx,Fit_End_idx,ProLen_m,...
    PixelNo,FitCheck,BC,MP_m)
ran(1)=Fit_Start_idx;
ran(2)=Fit_End_idx;

b(1)=C_gas;
b(2)=C_bg;
b(3)=t1;
b(4)=t2;
b(5)=ProLen_m;
b(6)=PixelNo;
% Mask=zeros(size(Xdata));
% Mask(Fit_Start_idx:Fit_End_idx)=1;
fun = @(p) sum((...
    ProfileData(ran(1):ran(2))-...
    BackDiffs_AutoCaller(p,b,ran,FitCheck,BC,MP_m)).^2,'omitnan');
%What about if you know D1 and k1 and are looking for D2 and k2?
if sum(FitCheck)==4
    pguess = [D1,D2,k1,k2];
elseif FitCheck(2)==0
    pguess = [D1,k1,k2];
elseif FitCheck(4)==0
    pguess = [D1,D2,k1];
end

[p,~] = fminsearch(fun,pguess,optimset('TolFun',1e-8));

% [p] = fmincon(fun,pguess,[],[],[],[],0.001*pguess,1000*pguess)
% [p,fminres] = fseminf(fun,pguess,0);
% [p,resnorm] = lsqcurvefit(@fun,pguess,BackDiffs_AutoCaller(p,b,ran,FitCheck),ProfileData(ran(1):ran(2)));
%

if sum(FitCheck)==4
    D1=p(1);D2=p(2);k1=p(3);k2=p(4);
elseif FitCheck(2)==0
    D1=p(1);D2=p(1);k1=p(2);k2=p(3);
elseif FitCheck(4)==0
    D1=p(1);D2=p(2);k1=p(3);k2=p(3);
end

function Align_CreateFcn(hObject, eventdata, hand)

function C_gas_text_CreateFcn(hObject, eventdata, hand)

function GridLines_ClickedCallback(hObject, eventdata, hand)
grid (get(hand.GridLines,'State'))
guidata(hObject, hand);

function D1_text_CreateFcn(hObject, eventdata, hand)

function ExportFigure_Callback(hObject, eventdata, hand)
%  [f p]=uiputfile('*.pdf','Save as PDF');
D1=str2double(get(hand.D1,'String'));
k1=str2double(get(hand.k1,'String'));
oldAxe=gca;
olfFig=gcf;
%

makeNew=1;
if isfield(hand,'newFig')
    if sum(ismember(findall(0,'type','figure'),hand.newFig))~=0
        makeNew=0;
    end
end

if makeNew==1
    hand.newFig=figure;
    set(hand.newFig,'Color',[1 1 1],...
        'WindowStyle','normal',...
        'PaperPositionMode','auto',...
        'PaperOrientation','landscape',...
        'Position',[60 50 1200 800],...
        'renderer','painters');
    
    % hc  = get(hand.MainAxes,'children')
    % hgc = get(hc, 'children')
    axesObject2=copyobj(oldAxe,hand.newFig);
    hline=findobj(gcf, 'type', 'line');
    % legend(hline([1,end-1,end]),{'Exchange Profiles','Max. Surface IF','Max. Peak Depth'}) % change order of legend entries
    set(hline,'LineWidth',1.5);
    
    set(legend,'Position',[0.67 0.82 0.10 0.03])
    set(gca,'fontunits','points')
    switch hand.CurrentPlot
        case '2Dmap'
            colormap(hot);
            set(gca,'position',[0.05 0.12 0.8 0.82])
            hand.colo=colorbar; ylabel(hand.colo,'Counts');
            set(hand.colo, 'TickLabelInterpreter','latex')
            set(hand.colo,'fontsize',24)
            set(hand.colo.Label,'interpreter','latex')
        case '2Dmap_norm'
            colormap(hot);
            set(gca,'position',[0.05 0.12 0.8 0.82])
            hand.colo=colorbar; ylabel(hand.colo,'Isotopic Fraction');
            set(hand.colo, 'TickLabelInterpreter','latex')
            set(hand.colo,'fontsize',24)
            set(hand.colo.Label,'interpreter','latex')
        case 'Mask'
        case 'DataPlotOnly'
        case 'CoupledUncertainty'
            set(gca,'position',[0.14 0.12 0.8 0.82])
            set(hline(end),'LineWidth',2)
            names={'Data','Fits'};
            order=[1 length(hline)];
            legend(hline(order),names);
            set(legend,'Interpreter','latex');
            set(legend,'Position',[0.75 0.82 0.10 0.03])
            set(legend,'Box','off')
            for i=1:length(hline)
                set(hline(i),'Color',[.1 .6 1],'Linewidth',0.5);
            end
            set(hline(1),'Color',[1 .1 .1],'Linewidth',1.5);
            if length(get(hand.XprimeTog,'State'))==2
                xlabel('Normalised depth, $x''$');
                set(gca,'xtick',[0:0.5:3]);
                %         set(gca,'xlables',[0:0.5:3]);
            else
                xlabel('Normalised Depth, $x''$');
            end
            if length(get(hand.CprimeY,'State'))==3
                ylabel('Isotopic Fraction, $C$');
            else
                ylabel('Normalised Isotopic Fraction, $C''$');
            end
            set(gca,'YColor',[0.2 0.2 0.2],...
                'XColor',[0.2 0.2 0.2]);
            uistack(hline(end), 'top')
            xlim([0 3])
        case 'Contourf'
            set(gca,'position',[0.05 0.12 0.8 0.82])
            hand.colo=colorbar; ylabel(hand.colo,'Standard Deviation of Fit Residuals');
            set(hand.colo, 'XTick', [0, 0.002,0.004,0.006,0.008, 0.01])
            set(hand.colo, 'XTickLabel', {'0', '0.002','0.004','0.006','0.008', '0.01+'})
            set(hand.colo, 'TickLabelInterpreter','latex')
            set(hand.colo,'fontsize',24)
            set(hand.colo.Label,'interpreter','latex')
            colormap(flipud(jet))
            %
            xlabel(['Self diffusivity, $D^*$ / 10$^{',num2str(round(log10(D1))),'}$m$^2$s$^{-1}$'])
            ylabel(['Effective exchange coefficient, $k^*$ / 10$^{',num2str(round(log10(k1))),'}$m s$^{-1}$'])
            axis square
            set(legend,'Interpreter','latex');
            set(get(gca,'xlabel'),'Interpreter','Latex');
            set(get(gca,'ylabel'),'Interpreter','Latex');
        case 'Generate'
            colormap(hot);
            set(gca,'position',[0.05 0.12 0.8 0.82])
            set(hline,'LineWidth',2.5)
            delete(hline(1))
            set(hline(2),'color',[0 1 0],...
                'linewidth',1.5,...
                'linestyle','-');
            set(gca,'xcolor',[0 0 0])
            set(gca,'ycolor',[0 0 0])
            xlabel('Depth, $x$ /$\mu$m');
            ylabel('Isotopic Fraction, $C$');
            legend('Isotopic Fraction','Surface Position')
            set(legend,'Interpreter','latex');
            set(legend,'Position',[0.49 0.83 0.21 0.08])
            set(legend,'color',[0.8 0.8 0.8]);
            set(legend,'edgecolor',[0.8 0.8 0.8]);
            set(legend,'fontsize',24);
            colo=colorbar; ylabel(colo,'Isotopic Fraction, $C$');
            set(colo,...%'Interpreter','latex',...
                'TickLabelInterpreter','latex',...
                'fontsize',24);
            %         set(gcf,'renderer','opengl')
            %         set(colo,'Position',[.82 .12 .03 .63]);
            %         set(legend,'Box','off')
            set(colo.Label,'interpreter','latex')
        case 'PlotButton'
            set(gca,'position',[0.14 0.12 0.8 0.82])
            if length(hline)==1
                set(hline,'LineWidth',1)
            else
                set(hline(1:2),'LineWidth',1)
                set(hline(end-1),'LineWidth',1.5)
            end
            set(hline(end),'MarkerSize',5)
            set(hline(end),'LineWidth',1)
            
            if get(hand.ErrorCheck, 'Value')==0
                legend('Data','Simulation')
            else
                legend('Data','Simulation','Residual')
            end
            set(legend,'Interpreter','latex');
            set(legend,'Position',[0.75 0.82 0.10 0.03])
            set(legend,'Box','off')
            xlabel('Depth /$\mu$m');
            if length(get(hand.CprimeY,'State'))==3
                ylabel('Isotopic Fraction, C');
            else
                ylabel('Normalised Isotopic Fraction, $C''$');
            end
            set(get(gca,'xlabel'),'Interpreter','Latex');
            set(get(gca,'ylabel'),'Interpreter','Latex');
            if length(get(hand.XprimeTog,'State'))==2
                xlabel('Normalised depth, $x''$');
                set(gca,'xtick',[0:0.5:3]);
                %         set(gca,'xlables',[0:0.5:3]);
            else
                xlabel('Depth, $x$ /$\mu$m');
            end
            if length(get(hand.GBplot,'State'))==2
                ylabel('Log. Normalised Isotopic Fraction, ln$|C''|$');
                xlabel('Normalised Depth, $\eta^{6/5}$');
                set(gca,'xtick',[0:2:10]);
            end
        case 'Fit'
            set(gca,'position',[0.14 0.12 0.8 0.82])
            set(hline(1:2),'LineWidth',1)
            set(hline(end-1),'LineWidth',1.5)
            set(hline(end),'MarkerSize',5)
            set(hline(end),'LineWidth',1)
            if get(hand.ErrorCheck, 'Value')==0;
                legend('Data','Simulation')
            else
                legend('Data','Simulation','Residual')
            end
            set(legend,'Interpreter','latex');
            set(legend,'Position',[0.75 0.82 0.10 0.03])
            set(legend,'Box','off')
            if length(get(hand.XprimeTog,'State'))==2
                xlabel('Normalised Depth, $x''$');
                set(gca,'xtick',[0:0.5:3]);
                %         set(gca,'xlables',[0:0.5:3]);
            else
                xlabel('Depth, $x$ /$\mu$m');
            end
            if length(get(hand.CprimeY,'State'))==3
                ylabel('Isotopic Fraction, $C$');
            else
                ylabel('Normalised Isotopic Fraction, $C''$');
            end
            if length(get(hand.GBplot,'State'))==2
                ylabel('Log. Normalised Isotopic Fraction, ln$|C''|$');
                xlabel('Normalised Depth, $\eta^{6/5}$');
                set(gca,'xtick',[0:2:10]);
            end
            set(gca,'YColor',[0.2 0.2 0.2],...
                'XColor',[0.2 0.2 0.2]);
    end
    set(get(gca,'xlabel'),'Interpreter','Latex');
    set(get(gca,'ylabel'),'Interpreter','Latex');
    set(gca,'Parent',hand.newFig,...
        'LineWidth',1.5,...
        'FontSize',24,...
        'TickLabelInterpreter','latex');
    %
    if length(hline(end).XData)>1000
        set(hline(end),'Marker','.')
    end
    % set(gca,'FontUnits','Normalized','FontSize',0.05);
    % disp('You are now in control of the figure and variables (type "return" to exit)')
else
    hline=findobj(olfFig, 'type', 'line');
    %     hline=hline(1);
    figure(hand.newFig)
    new_handle = copyobj(hline,gca);
    %     ylim([0 inf])
    set(new_handle,'LineWidth',1.5);
end
guidata(hObject, hand);

function CprimeY_ClickedCallback(hObject, eventdata, hand)
if length(get(hand.CprimeY,'State'))==3;
    set(hand.GBplot,'State','off')
end
[hand] =PlotButton_Callback(hObject, eventdata, hand);
guidata(hObject, hand);

function XprimeTog_ClickedCallback(hObject, eventdata, hand)
if length(get(hand.XprimeTog,'State'))==2;
    set(hand.GBplot,'State','off')
end
[hand] =PlotButton_Callback(hObject, eventdata, hand);
guidata(hObject, hand);

function GBplot_ClickedCallback(hObject, eventdata, hand)
if length(get(hand.GBplot,'State'))==2;
    set(hand.CprimeY,'State','on')
    set(hand.XprimeTog,'State','off')
end
[hand] =PlotButton_Callback(hObject, eventdata, hand);
guidata(hObject, hand);

function uipanel2_ResizeFcn(hObject, eventdata, hand)

function MainAxes_ButtonDownFcn(hObject, eventdata, hand)

function uipanel7_ButtonDownFcn(hObject, eventdata, hand)
if ~isfield(hand,'Secrets')
    hand.Secrets=1;
else
    hand.Secrets= hand.Secrets+1;
    if  hand.Secrets==4
        set(hand.Advanced,'Visible','on')
        set(hand.Check_LeClaire,'Visible','on')
        
        %         set(hand.Build,'Visible','on')
    end
end
guidata(hObject, hand);

function Untitled_1_Callback(hObject, eventdata, hand)

function Go_Callback(hObject, eventdata, hand)
disp('Going...')
PixelNo=800;

% set(hand.t1,'String',1.4);
% set(hand.t2,'String',0.66);
% set(hand.D1,'String',1e-16);
% set(hand.D2,'String',1e-16);
% keyboard
y=-10:0.5:10;
k_range=2.^y;

for i=1:length(y)
    %     set(hand.k1,'String',(k_range(i)));
    set(hand.k2,'String',num2str(1.3e-08*(k_range(i))));
    [hand] =PlotButton_Callback(hObject, eventdata, hand);
    Profs(i,1:length(hand.pro))=hand.pro;
end

figure
hold on;plot(hand.X/(2*sqrt(1.7e-12*2.06*3600)),Profs);hold off
hline=findobj(gcf, 'type', 'line');
ylim([0 0.3])
% LineNo=length(hline);for i=1:LineNo;hline(i).Color=[0.2 i/LineNo 0.3];end
keyboard
%
%
% set(hand.PixelNo,'String',PixelNo);
% xStarLen=4;
% ProfileLength=round(xStarLen*(1e6*(4*1e-13*3600)^0.5),3,'significant');
% set(hand.ProfileLength,'String',ProfileLength);
% [hand] =PlotButton_Callback(hObject, eventdata, hand);
% tic
% global SurfConc
% global thetaFine
% global G_pro
% global G_pro3D
% global k2
% global G_x
% global G_xStar
% global G_ProData
% global G_Peak
% global G_xPeak_idx
% global theta
% D1=1e-13;
% D2=1e-13;
% k1=1e-5;
% k2=1e-5;
% set(hand.D1,'String',D1);
% set(hand.D2,'String',D2);
% set(hand.k1,'String',k1);
% set(hand.k2,'String',k2);
%
% t1=1;
% t2=0;
% set(hand.t1,'String',t1);
% set(hand.t2,'String',t2);
% C_bg=str2double(get(hand.C_bg,'String'));
% C_gas=str2double(get(hand.C_gas,'String'));
% t_tot=t1+t2;
%
%
% set(hand.PixelNo,'String',PixelNo);
% G_x=hand.X;
% G_xStar=G_x/(4*D1*t_tot*3600)^0.5;
%
% theta=[0:1/6:1];%[-3:0.1:3];
% ls=[-20:1];
% k2=k1*2.^ls;  %10.^ls;
% k2(1)=0;k2(ls==0)=k2(ls==0)*1.01;
% G_pro=zeros(length(k2),PixelNo);
% G_pro3D=zeros(length(theta),length(k2),PixelNo);
% for i=1:length(theta)
%     if theta(i)==0 %%% All In Diffusion
%         set(hand.t1,'String',1);
%         set(hand.t2,'String',0);
%         set(hand.k2,'String',k1*1.1);
%         [hand] =PlotButton_Callback(hObject, eventdata, hand);
%         G_pro(:,:)=meshgrid((hand.pro-C_bg)/(C_gas-C_bg),k2);
%     elseif theta(i)==1 %%% All Back Diffusion
%         G_pro=zeros(length(k2),PixelNo);
%     else
%         set(hand.t1,'String',t_tot*(1-theta(i)));
%         set(hand.t2,'String',t_tot*(theta(i)));
%         for j=1:length(k2)
%             set(hand.k2,'String',k2(j))
%             [hand] =PlotButton_Callback(hObject, eventdata, hand);
%             G_pro(j,:)=(hand.pro-C_bg)/(C_gas-C_bg);
%         end
%     end
%     G_pro3D(i,:,:)=G_pro;
% end
% toc
%
% thetaFine=[0:0.005:0.195,0.2:0.04:0.76,0.8:0.005:1];
% set(hand.k1,'String',1e-04);
% set(hand.k2,'String',0);
% for i=1:length(thetaFine)
%     set(hand.t1,'String',t_tot*(1-thetaFine(i)));
%     set(hand.t2,'String',t_tot*(thetaFine(i)));
%     [hand] =PlotButton_Callback(hObject, eventdata, hand);
%     SurfConc(i)=(hand.pro(1)-C_bg)/(C_gas-C_bg);
% end
% SurfConc(1)=1;SurfConc(end)=0; %correction for crappy erfc tables
% toc
% set(hand.k1,'String',1e-05);
% set(hand.k2,'String',1e-04);
% for i=1:length(thetaFine)
%     set(hand.t1,'String',t_tot*(1-thetaFine(i)));
%     set(hand.t2,'String',t_tot*(thetaFine(i)));
%     [hand] =PlotButton_Callback(hObject, eventdata, hand);
%     [G_xPeak_idx(i),G_xPeak_idx(i)]=max(hand.pro);
%     %     k1=1e-05;k2=1e-04;
%     %     t1=t_tot*(1-thetaFine(i));t2=t_tot*(thetaFine(i));
%     %     [X,pro,DepthFlag]=BackDiffsCN_inline(C_gas,C_bg,D1,D2,k1,k2,t1,t2,ProfileLength,PixelNo);
%     %     [G_xPeak_idx(i),G_xPeak_idx(i)]=max(pro);
% end
%
% %%% Plotting
% figure('Color',[1 1 1]);
% plot31=plot3(G_xStar',zeros(length(G_xStar)),permute(G_pro3D(1,1,:),[3 1 2]));
%
% G_xPeak_idx(1)=1;G_xPeak_idx(end)=G_xPeak_idx(end-1);
% plot3(G_xStar(G_xPeak_idx)',thetaFine,meshgrid(0,thetaFine),'linewidth',2);hold on
% plot3(meshgrid(0,thetaFine)',meshgrid(thetaFine,0),SurfConc,'linewidth',2)
% for k=1:length(k2)
%     if k==1 || k==length(k2)
%         hold on; plot3(meshgrid(G_xStar,theta)',meshgrid(theta,G_xStar),...
%             permute(G_pro3D(:,k,:),[3 1 2]),'k','linewidth',2);
%     else
%         hold on; plot3(meshgrid(G_xStar,theta)',meshgrid(theta,G_xStar),...
%             permute(G_pro3D(:,k,:),[3 1 2]),'k');
%     end
% end
% xlabel('Normalised Depth, x''','Rotation',-8);
% ylabel('Exchange Time Ratio, \theta','Rotation',11);
% zlabel('Normalised Concentration, C''');box on
% set(gca,'YTick',theta);
% set(gca,'YTickLabel',{'0','1/6','1/3','1/2','2/3','5/6','1'});
% set(gca,'ZTick',[0 1]);
% set(gca,'ZTickLabel',{'0','a_0'});
% xlim([0 2.5]);ylim([0 1]);zlim([0 1]);
% hold off
% set(gca,'YDir','Reverse');
%
% Leg={'Max. Peak Depth','Max. Surface Conc.','Exchange Profiles'};
% legend(Leg,'Position',[.72 .67 .10 .085]);
% set(gca,'FontSize',16);
% ax=gca;
% ax.XLabel.Position = [1.3 1 -.1];
% ax.YLabel.Position = [2.6 .4 -.1];
% view(45, 15);
% annotation(gcf,'textarrow',[0.57 0.5],[0.48 0.510],...
%     'String',{'\lambda=\infty'},'LineWidth',1,...
%     'HeadWidth',6,'HeadStyle','cback1',...
%     'HeadLength',6,'FontWeight','bold',...
%     'FontSize',16);
% annotation(gcf,'textarrow',[0.567 0.537],[0.549 0.563],...
%     'String',{'\lambda=0'},'LineWidth',1,...
%     'HeadWidth',6,'HeadStyle','cback1',...
%     'HeadLength',6,'FontWeight','bold',...
%     'FontSize',16);
% set(gcf,'Position', [100, 100, 1049, 895]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%export_fig('D:\PhD\BackDiffusion\MatLab Figures\OmniGraphHex_ef.pdf')
% set(gcf,'Legend','String',
% set(plot31(240),'DisplayName','Max Peak Depth, x*_{peak}');
% set(plot31(241),'DisplayName','Max. Surface Conc., C''_{max}');
% set(plot31(242),'DisplayName','Back Exchange Profiles_{}^{}','LineWidth',2);

% figure(11);plot(thetaFine,SurfConc(:,1)/max(SurfConc(:,1)));ylim([0 1]);
% plot(thetaFine,0.5*SurfConc(:,1)/SurfConc((1+end)/2,1),thetaFine,acos((thetaFine*2)-1)/pi());
% ylim([0 1]);set(gca,'xTick',([0:0.1:1]));grid;axis square
%%% divide everything everywhere by max(Crank(theta=0))
% k2_start=1e-11;
% ls=[-10:10];%[-3:0.1:3];
% k2=k1*2.^ls;%10.^ls;
% % theta=[0:0.01:1];
% % t_tot=100;
% % t1=[10:10:100];
% set(hand.k2,'String',k2_start)
% % for i=1:length(t1)
% % %     set(hand.t1,'String',t1(i));
% % for i=1:length(theta)
% %     set(hand.t1,'String',t_tot*(1-theta(i)))
% %     set(hand.t2,'String',t_tot*(theta(i)))
% for i=1:length(k2)
% %         k2(i)=k2_start*1.1^(i-1);
% %         k2(i)=50*i*k2_start;
%         set(hand.k2,'String',k2(i))
%     [hand] =PlotButton_Callback(hObject, eventdata, hand);
%     [G_Peak(i,1),G_Peak(i,2)]=max((hand.pro-C_bg)/(C_gas-C_bg));
%     SurfConc(i)=(hand.pro(1)-C_bg)/(C_gas-C_bg);
%     G_pro(i,:)=(hand.pro-C_bg)/(C_gas-C_bg);
%
%     %     set(hand.k2,'String',1e-12+(i-1)*1e-13)
%
% end
%
% % global G_x; global k2; global G_pro; global SurfConc;kRAT=k2/4.4e-10;
% % surf(G_x*1e6, kRAT, G_pro);xlabel('Depth /um');ylabel('Ratio k2/k1');zlabel('C''');
% figure; plot(theta,G_Peak(:,1),'k');xlabel('Theta');ylabel('C''_{peak}');
% figure; plot(1e6*G_x(1:308),G_pro(:,1:308),'k');xlabel('NormalisedDepth /um');ylabel('C''');
% hold on;plot(1e6*G_x(1:308),G_pro(61,1:308),'g:','LineWidth',1.5)
% hold on;plot(1e6*G_x(1:308),G_pro(41,1:308),'r:','LineWidth',1.5)

% figure; plot(G_xStar,G_pro,'k');xlabel('Normalised Depth, x*');ylabel('Normalised Concentration, C''');
% hold on;plot(G_xStar,G_pro(31,:),'r:','LineWidth',1.5)
% hold on;plot(G_xStar,G_pro(51,:),'g:','LineWidth',1.5)
toc
sound(1,1000);pause(0.2);sound(1,1000);
disp('Gone')
guidata(hObject, hand);

function Build_Callback(hObject, eventdata, hand)
[hand] =PlotButton_Callback(hObject, eventdata, hand);
% delete(gca)
% hand.AlignAngle_FiAl_num=0;%So that align isn't auto called

set(hand.AlignAngle,'String',0);
pro=zeros(size(hand.pro));
SurfPos=str2double(get(hand.SurfPos, 'String'));
ProLen=str2double(get(hand.ProfileLength, 'String'));
PixelNo=str2double(get(hand.PixelNo, 'String'));

SurfPos_idx=1+round(SurfPos/(ProLen/(PixelNo-1)));
pro(SurfPos_idx:end)=hand.pro(1:end-SurfPos_idx+1);
counts=100;
absErr=str2double(get(hand.absErr,'String'));
relErr=str2double(get(hand.relErr,'String'));

pro18=counts*pro;
% pro16=pro18/pro-pro18;
pro16=counts*ones(size(pro))-pro18;
pro16(pro==0)=0;

hand.O16Flag=1;hand.O18Flag=1;

hand.O18image=...
    (1-relErr*(1-2*rand(length(pro18)))).*...%
    (absErr*counts*(1-2*rand(length(pro18)))+...
    meshgrid(pro18,pro18));
hand.O18image(hand.O18image<0)=0;
hand.O18image_Or=hand.O18image;

hand.O16image=(1-relErr*2*(1-2*rand(length(pro16)))).*...
    (absErr*counts*2*(1-2*rand(length(pro16)))+...
    meshgrid(pro16,pro16));
hand.O16image(hand.O16image<0)=0;
hand.O16image_Or=hand.O16image;


hand.NormO18image_Or=hand.O18image./(hand.O18image+hand.O16image);
hand.O16pO18image_Or=hand.O18image+hand.O16image;

set(hand.ProfDataSaveName,'String','Dummy');
hand.PathName=[cd,'\'];
hand.FileName='Dummy';
hand.SaveName_Or='Dummy';
% imagepro=meshgrid(pro,pro);

% imagesc(imagepro);
% colormap(hot);
% hand.colo=colorbar; ylabel(hand.colo,'Counts');
% axis square; set(gca, 'XTick', [],'YTick', []);
% set(hand.Xlimit,'Visible','off');set(hand.Ylimit,'Visible','off');
% set(hand.Fit_End,'String',roundsf(...
%     0.5*str2double(get(hand.ProfileLength,'String')),3,'floor'));
% pause(0.5)

si=[0.35 0.35];
sp(1)=subplot (3,2,3); imagesc(hand.O16image);
set(gca,'position',[0.03 0.45 si]);title('O16');
sp(2)=subplot (3,2,4); imagesc(hand.O18image);
set(gca,'position',[0.4 0.45 si]);title('O18');
sp(3)=subplot (3,2,5); imagesc(hand.O16pO18image_Or);
set(gca,'position',[0.03 0.03 si]);title('O16 + O18');
sp(4)=subplot (3,2,6); imagesc(hand.NormO18image_Or);
set(gca,'position',[0.4 0.03 si]);title('O18 Norm.');
% set(gcf,'position',hand.ImagePlotPos);
set(sp, 'XTick', [],'YTick', []);
set(hand.Xlimit,'Visible','off');set(hand.Ylimit,'Visible','off');
axis(sp,'square');colormap(hot);
disp('Built!')
waitforbuttonpress
delete(sp)
%
% set(sp,'position',[.01  .1  .3 .3]);

% colormap(hot);
% hand.colo=colorbar; ylabel(hand.colo,'Counts');

% set(hand.Xlimit,'Visible','off');set(hand.Ylimit,'Visible','off');
%
% pause
% imagesc(imagepro-image18./(image16+image18))
% colormap(hot);
% hand.colo=colorbar; ylabel(hand.colo,'Counts');
% axis square; set(gca, 'XTick', [],'YTick', []);
% set(hand.Xlimit,'Visible','off');set(hand.Ylimit,'Visible','off');

guidata(hObject, hand);

function xStar_d_ButtonDownFcn(hObject, eventdata, hand)

function Xstar_ButtonDownFcn(hObject, eventdata, hand)

function uipanel2_ButtonDownFcn(hObject, eventdata, hand)
% hObject    handle to uipanel2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% hand    structure with hand and user data (see GUIDATA)
D1=str2num(get(hand.D1,'String'));
t1=str2num(get(hand.t1,'String'));
t2=str2num(get(hand.t2,'String'));
t_tot=3600*(t1+t2);
set(hand.Fit_End,'String',roundsf(4e6*sqrt(D1*t_tot),3,'round'));
set(hand.xStar_d,'String',2);
guidata(hObject, hand);

function absErr_Callback(hObject, eventdata, hand)

function absErr_CreateFcn(hObject, eventdata, hand)
% hObject    handle to absErr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% hand    empty - hand not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function relErr_Callback(hObject, eventdata, hand)

function relErr_CreateFcn(hObject, eventdata, hand)
% hObject    handle to relErr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% hand    empty - hand not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Uncertainty_Callback(hObject, eventdata, hand)
set(hand.Uncertainty,'String','Calculating...');pause(0.001);
% hObject    handle to Uncertainty (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% hand    structure with hand and user data (see GUIDATA)
C_bg=str2double(get(hand.C_bg, 'String'));
C_gas=str2double(get(hand.C_gas, 'String'));
D1=str2double(get(hand.D1, 'String'));
D2=str2double(get(hand.D2, 'String'));
k1=str2double(get(hand.k1, 'String'));
k2=str2double(get(hand.k2, 'String'));
t1=3600*str2double(get(hand.t1, 'String'));
t2=3600*str2double(get(hand.t2, 'String'));
ProfileLength=str2double(get(hand.ProfileLength, 'String'));
PixelNo=str2double(get(hand.PixelNo, 'String'));
t_tot=(t1+t2) ;
theta=t2/(t1+t2);
if get(hand.k2FitCheck, 'Value')==1
    %k2 and k1
    ls1=[-1:0.2:1];
    k1_n=k1*10.^ls1;
    ls2=[-2:0.4:2];
    k2_n=k2*10.^ls2;
    for i=1:length(k2_n)
        for j=1:length(k1_n)
            set(hand.k2,'String',k2_n(i));
            set(hand.k1,'String',k1_n(j));
            [hand] =PlotButton_Callback(hObject, eventdata, hand);
            RSqMap(i,j)=hand.r2;
            RMSeMap(i,j)=hand.rmse;
        end
        disp([num2str(i*length(ls1)),'/',num2str(length(ls1)*length(ls2))])
    end
    set(hand.k1,'String',roundsf(k1,3,'round'));
    set(hand.k2,'String',roundsf(k2,3,'round'));
    [hand] =PlotButton_Callback(hObject, eventdata, hand);
    set(hand.Xlimit,'Visible','off');
    set(hand.Ylimit,'Visible','off');
    k1_nI=linspace(min(k1_n),max(k1_n),200);
    k2_nI=linspace(min(k2_n),max(k2_n),200);
    contourf(k1_nI,k2_nI,interp2(k1_n,k2_n,RMSeMap,k1_nI,k2_nI'),64,'LineStyle','none')
    %     contourf(meshgrid(k1_n,k2_n),meshgrid(k2_n,k1_n)',RMSeMap)
    hand.colo=colorbar; ylabel(hand.colo,'Standard Deviation of Fit Residuals');
    set(hand.colo,'FontSize',10);
    caxis([0,2e-2]);
    set(hand.colo, 'XTick', [0, 0.004,0.008,0.012,0.016, 0.02])
    set(hand.colo, 'XTickLabel', {'0', '0.004','0.008','0.0012','0.0016', '0.02+'})
    set(gca,'position',hand.ImagePlotPos);
    colormap(flipud(parula))
    set(gca,'XScale','log');set(gca,'YScale','log');
    xlabel('Effective exchange coefficient, k1* / m s^{-1}')
    ylabel('Effective exchange coefficient, k2* / m s^{-1}')
    axis square
    
else
    %D1 and k1
    % should you change the fit region as D varies?????
    %
    ls1=[0:0.2:2];
    D1_n=D1*ls1;
    ls2=[0:0.2:2];
    k1_n=k1*ls2;
    for i=1:length(k1_n)
        for j=1:length(D1_n)
            set(hand.k1,'String',k1_n(i));
            set(hand.D1,'String',D1_n(j));
            [hand] =PlotButton_Callback(hObject, eventdata, hand);
            RSqMap(i,j)=hand.r2;
            RMSeMap(i,j)=hand.rmse;
        end
        disp([num2str(i*length(ls1)),'/',num2str(length(ls1)*length(ls2)),' residuals quantified'])
    end
    set(hand.D1,'String',roundsf(D1,3,'round'));
    set(hand.k1,'String',roundsf(k1,3,'round'));
    [hand] =PlotButton_Callback(hObject, eventdata, hand);
    set(hand.Xlimit,'Visible','off');
    set(hand.Ylimit,'Visible','off');
    
    %% Calculate the coupled
    ProTemp_p=(hand.ProfileData(1:hand.Fit_End_idx)-C_bg)/(C_gas-C_bg);
    XTemp=hand.X(1:hand.Fit_End_idx);
    %
    if t2==0
        for i=1:length(hand.ProfileData(1:hand.Fit_End_idx))
            funCoupledDk = @(g) ((g-D1)/D1)^2+...
                ((Find_k_from_D_at_pCrank(g,t_tot,ProTemp_p(i),XTemp(i))-k1)/k1)^2;
            
            %% Optimise
            pguess = [D1];
            [g,fminres] = fminsearch(funCoupledDk,pguess,optimset('TolFun',1e-8));
            Dandk(i,1)=[g];
            Dandk(i,2)=Find_k_from_D_at_pCrank(g,t_tot,ProTemp_p(i),XTemp(i));
            NewCranks(i,:)=C_bg+(C_gas-C_bg)*...
                (erfc(hand.X./(2*sqrt(g(1)*t_tot)))-...
                exp(Dandk(i,2)/g(1).*hand.X+t_tot*Dandk(i,2)^2/g(1)).*erfc(hand.X./(2*sqrt(g(1)*t_tot))+Dandk(i,2)*sqrt(t_tot/g(1))));
            h=plot(hand.X/(2*sqrt(D1*t_tot)),NewCranks');
            hold on;plot(hand.Xdata(1:i)/(2*sqrt(D1*t_tot)),hand.ProfileData(1:i),'k','linewidth',1.5);hold off;
            pause(0.01)
        end
    else
        for i=1:length(hand.ProfileData(1:hand.Fit_End_idx))
            funCoupledDk = @(g) ((g-D1)/D1)^2+...
                ((Find_k_from_D_at_pBackCrank(g,t_tot,theta,ProTemp_p(i),XTemp(i))-k1)/k1)^2;
            %                         funCoupledDk = @(g) (log(g/D1))^2+...
            %                             (log(Find_k_from_D_at_pBackCrank(g,t_tot,theta,ProTemp_p(i),XTemp(i))/k1))^2;
            funCoupledDk = @(g) sum((ProTemp_p-(...
                (erfc(XTemp./(2*sqrt(g*t_tot)))-...
                exp(Find_k_from_D_at_pBackCrank(g,t_tot,theta,ProTemp_p(i),XTemp(i))/g.*XTemp+t_tot*Find_k_from_D_at_pBackCrank(g,t_tot,theta,ProTemp_p(i),XTemp(i))^2/g).*...
                erfc(XTemp./(2*sqrt(g*t_tot))+Find_k_from_D_at_pBackCrank(g,t_tot,theta,ProTemp_p(i),XTemp(i))*sqrt(t_tot/g)))-...
                (erfc(XTemp./(2*sqrt(g*t_tot*theta)))-...
                exp(Find_k_from_D_at_pBackCrank(g,t_tot,theta,ProTemp_p(i),XTemp(i))/g.*XTemp+t_tot*theta*Find_k_from_D_at_pBackCrank(g,t_tot,theta,ProTemp_p(i),XTemp(i))^2/g).*...
                erfc(XTemp./(2*sqrt(g*t_tot*theta))+Find_k_from_D_at_pBackCrank(g,t_tot,theta,ProTemp_p(i),XTemp(i))*sqrt(t_tot*theta/g)))...
                )).^2,'omitnan');
            
            %% Optimise
            %             pguess = [D1];
            %             [g,fminres] = fminsearch(funCoupledDk,pguess,optimset('TolFun',1e-30,'TolX',1e-30,'MaxFunEvals',1000,'MaxIter',1000));%'TolX',1e-18,
            [g,fminres] = fminbnd(funCoupledDk,.1*D1,10*D1,optimset('TolFun',1e-18,'TolX',1e-18));%,'MaxFunEvals',1000,'MaxIter',1000));%'TolX',1e-18,
            Dandk(i,1)=[g];
            Dandk(i,2)=Find_k_from_D_at_pBackCrank(g,t_tot,theta,ProTemp_p(i),XTemp(i));
            NewCranks(i,:)=C_bg+(C_gas-C_bg)*(...
                (erfc(hand.X./(2*sqrt(g*t_tot)))-...
                exp(Dandk(i,2)/g.*hand.X+t_tot*Dandk(i,2)^2/g(1)).*erfc(hand.X./(2*sqrt(g*t_tot))+Dandk(i,2)*sqrt(t_tot/g)))-...
                (erfc(hand.X./(2*sqrt(g*t_tot*theta)))-...
                exp(Dandk(i,2)/g.*hand.X+t_tot*theta*Dandk(i,2)^2/g).*erfc(hand.X./(2*sqrt(g*t_tot*theta))+Dandk(i,2)*sqrt(t_tot*theta/g)))...
                );
            h=plot(hand.X/(2*sqrt(D1*t_tot)),NewCranks');xlim([0 3]);
            hold on;plot(hand.Xdata(1:i)/(2*sqrt(D1*t_tot)),hand.ProfileData(1:i),'k','linewidth',1.5);hold off;
            pause(0.01)
            %             Dandk0=meshgrid([D1,k1],Dandk(:,1));subplot(1,2,1);plot((Dandk-Dandk0)./Dandk0);subplot(1,2,2);plot(sum( ((Dandk-Dandk0)./Dandk0).^2 ,2))
        end
    end
    %
    sorted_Dandk=sort(Dandk);
    RelevantPoints=length(hand.X(1:hand.Fit_End_idx));
    
    D_minus=D1-sorted_Dandk(ceil(RelevantPoints*0.025),1);
    D_plus=sorted_Dandk(floor(RelevantPoints*0.975),1)-D1;
    
    disp(['D1 = ',num2str(round(D1,2,'significant')),' - ',num2str(round(D_minus,2,'significant')),' + ',num2str(round(D_plus,2,'significant'))])
    
    k_minus=k1-sorted_Dandk(ceil(RelevantPoints*0.025),2);
    k_plus=sorted_Dandk(floor(RelevantPoints*0.975),2)-k1;
    
    disp(['k1 = ',num2str(round(k1,2,'significant')),' - ',num2str(round(k_minus,2,'significant')),' + ',num2str(round(k_plus,2,'significant'))])
    %% Plotting
    
    %     set(gca,'position',[0.03 0.03 0.4 0.4])
    hand.CurrentPlot='Contourf';
    scaleD1=round(log10(D1));
    scalek1=round(log10(k1));
    contourf(D1_n/10^scaleD1,k1_n/10^scalek1,RMSeMap,32,'LineStyle',':')
    hand.colo=colorbar; ylabel(hand.colo,'Standard Deviation of Fit Residuals');
    set(hand.colo,'FontSize',10);
    caxis([0,1e-2]);
    set(hand.colo, 'XTick', [0, 0.002,0.004,0.006,0.008, 0.01])
    set(hand.colo, 'XTickLabel', {'0', '0.002','0.004','0.006','0.008', '0.01+'})
    set(gca,'position',hand.ImagePlotPos);
    colormap(flipud(jet))
    xlabel(['Self Diffusivity, D* / 1e',num2str(round(log10(D1))),'m^2s^{-1}'])
    ylabel(['Effective exchange coefficient, k* / 1e',num2str(round(log10(k1))),'m s^{-1}'])
    axis square
    guidata(hObject, hand);
    %
    waitforbuttonpress
    pause(0.1)
    waitforbuttonpress
    h=plot(hand.X/(2*sqrt(D1*t_tot)),NewCranks',hand.Xdata/(2*sqrt(D1*t_tot)),hand.ProfileData,'k');
    %     hold on;plot();hold off
    set(gca,'position',hand.GraphPlotPos);
    %     title(['Curves required to fit each point up to x*=',get(hand.xStar_d,'String')]);
    xlabel('Normalised Depth, x''');ylabel('Isotopic Fraction');
    hand.CurrentPlot='CoupledUncertainty';
    guidata(hObject, hand);
    %
    guidata(hObject, hand);
    waitforbuttonpress
    pause(0.1)
    waitforbuttonpress
    PlotButton_Callback(hObject, eventdata, hand);
    %     set(gca,'position',[0.03 0.45 0.4 0.4])
end
sound(1,1000);pause(0.2);sound(1,1000);
set(hand.Uncertainty,'String','Uncertainty')

function [k]=Find_k_from_D_at_pCrank(D,t_tot,pointC,pointX);

% pointC=(pointC-0.002)/(0.95-0.002);
if D<=0
    k=10;
else
    A=erfc(pointX/(2*sqrt(D*t_tot)));
    fun_k_from_D = @(g) (pointC-...
        (A-exp(g/D.*pointX+t_tot*g^2/D).*erfc(pointX./(2*sqrt(D*t_tot))+g*sqrt(t_tot/D)))...
        )^2;
    [k,fminres] = fminbnd(fun_k_from_D,0,sqrt(26^2*D/t_tot),optimset('TolX',1e-18,'TolFun',1e-18));
end

function [k]=Find_k_from_D_at_pBackCrank(D,t_tot,theta,pointC,pointX);

% pointC=(pointC-0.002)/(0.95-0.002);
if D<=0
    k=10;
else
    A=erfc(pointX/(2*sqrt(D*t_tot)));
    B=erfc(pointX/(2*sqrt(D*t_tot*theta)));
    t_222=t_tot*theta;
    fun_k_from_D = @(g) (pointC-(...
        (A-exp(g/D.*pointX+t_tot*g^2/D).*erfc(pointX./(2*sqrt(D*t_tot))+g*sqrt(t_tot/D)))-...
        (B-exp(g/D.*pointX+t_222*g^2/D).*erfc(pointX./(2*sqrt(D*t_222))+g*sqrt(t_222/D)))...
        ))^2;
    
    [k,fminres] = fminbnd(fun_k_from_D,0,sqrt(26^2*D/t_tot),optimset('TolX',1e-18,'TolFun',1e-18));
end

function PlotO16image_CreateFcn(hObject, eventdata, hand)

function BoundCondMode_Callback(hObject, eventdata, hand)
switch get(hand.BoundCondMode,'Value')
    case 1
        set(hand.MirrorPlane,'Visible','off');
        set(hand.Plane_text,'Visible','off');
    case 2
        set(hand.MirrorPlane,'Visible','on');
        set(hand.Plane_text,'Visible','on');
    case 3
        set(hand.MirrorPlane,'Visible','on');
        set(hand.Plane_text,'Visible','on');
end
[hand] =PlotButton_Callback(hObject, eventdata, hand);%fun

function MirrorPlane_Callback(hObject, eventdata, hand)
[hand] =PlotButton_Callback(hObject, eventdata, hand);

function MirrorPlane_CreateFcn(hObject, eventdata, hand)
% hObject    handle to MirrorPlane (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% hand    empty - hand not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function PlotProperties(hObject, eventdata, hand)
set(gca,...
    'LineWidth',1,...
    'FontUnits','Normalized',...
    'FontSize',0.034);
% set(get(gca,'xlabel'),'FontSize','Latex');
% set(get(gca,'ylabel'),'Interpreter','Latex');

guidata(hObject, hand);

function Reset_Callback(hObject, eventdata, hand)
% close(TraceX)
a=hand.reset;
hand=1;
hand=a;
hand.reset=hand;
set(hand.AlignAngle,'String','0');
set(hand.MaskThresh,'String','0.1');
set(hand.SurfPos,'String',0);
set(hand.D1,'String','1e-13');
set(hand.D2,'String','1e-13');
set(hand.k1,'String','1e-9');
set(hand.k2,'String','1e-9');
set(hand.t1,'String','1');
set(hand.t2,'String','0');
set(hand.C_bg,'String','0.002');
set(hand.C_gas,'String','1');
set(hand.ProfileLength,'String','500');
set(hand.PixelNo,'String','500');
set(hand.Fit_Start,'String','0')
set(hand.Fit_End,'String','100');
set(hand.AlignAngle,'BackgroundColor',[1 1 1]);
set(hand.Fit_Start,'BackgroundColor',[1 1 1]);
set(hand.SurfPos,'BackgroundColor',[1 1 1]);
set(hand.ProfileLength,'BackgroundColor',[1 1 1]);
set(hand.AlignAngle,'BackgroundColor',[1 1 1]);
set(hand.PixelNo,'BackgroundColor',[1 1 1]);
set(hand.C_bg,'BackgroundColor',[1 1 1]);
set(hand.Ylimit,'Visible','off');
set(hand.Xlimit,'Visible','off');
set(gca,'Position',[0.0782    0.0928    0.6651    0.7078])
[A, map, alpha] = imread('TraceX_logo.png');
h = imshow(A, map);
set(h, 'AlphaData', alpha);

guidata(hObject, hand);

function [hand]=Check_Compress_Callback(hObject, eventdata, hand)
if ~isfield(hand,'ProfileData_Or')
    [hand]=LoadProfileData_Callback(hObject, eventdata, hand);
end
%
if get(hand.Check_Compress,'Value')==1
    method = 'linear';
    PixelNo=length(hand.ProfileData_Or);%str2double(get(hand.PixelNo,'String'));
    ProLen=str2double(get(hand.ProfileLength,'String'));
    switch method
        case 'linear'
            X=linspace(0,ProLen,length(hand.ProfileData_Fund));
            Y=hand.ProfileData_Fund;
            Z=linspace(0,ProLen,str2double(get(hand.Edit_Compress,'String')));
            hand.ProfileData_Or=interp1(X,Y,Z);
            set(hand.PixelNo,'String',length(hand.ProfileData_Or));
        case 'discard'
            target=str2double(get(hand.Edit_Compress,'String'));
            skip=round(PixelNo/target);
            dx_Or=ProLen/(PixelNo-1);
            hand.ProfileData_Or=hand.ProfileData_Fund(1:skip:end);
            set(hand.PixelNo,'String',length(hand.ProfileData_Or));
            set(hand.ProfileLength,'String',dx_Or*skip*(length(hand.ProfileData_Or)-1));
    end
else
    hand.ProfileData_Or=hand.ProfileData_Fund;
    set(hand.PixelNo,'String',length(hand.ProfileData_Or));
end
[hand]=DataPlotOnly(hObject, eventdata, hand);

function Edit_Compress_Callback(hObject, eventdata, hand)
if get(hand.Check_Compress,'Value')==1
    Check_Compress_Callback(hObject, eventdata, hand);
end

function Edit_Compress_CreateFcn(hObject, eventdata, hand)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function [hand]=BuildX(hObject,hand)
ProLen=str2double(get(hand.ProfileLength, 'String'))*1e-6;
SurfPos=str2double(get(hand.SurfPos, 'String'))*1e-6;
PixelNo=str2double(get(hand.PixelNo, 'String'));
X=linspace(0,ProLen,PixelNo);
[~, hand.SurfPos_idx] = min(abs(X-SurfPos));
hand.Xdata=X(1:end-hand.SurfPos_idx+1);
hand.ProfileData=hand.ProfileData_Or(hand.SurfPos_idx:end);
hand.Xdata_Or=X;
guidata(hObject,hand);

function [hand]=Approx_Dandk(hObject,hand)
t_tot=3600*(str2num(get(hand.t1,'String'))+str2num(get(hand.t2,'String')));
ProLen=str2double(get(hand.ProfileLength, 'String'))*1e-6;
if isfield(hand,'ProfileData_Or')
    C_max=max(hand.ProfileData_Or(isfinite(hand.ProfileData_Or)));
else
    C_max=0.5;
end
D_approx=roundsf(0.25*((ProLen/2)^2/t_tot),2,'round');
k_approx=roundsf(0.25*abs(sqrt(D_approx/(t_tot*pi)))/(1-C_max),2,'round');
set(hand.D1,'String',num2str(D_approx));
set(hand.D2,'String',num2str(D_approx));
set(hand.k1,'String',num2str(k_approx));
set(hand.k2,'String',num2str(k_approx));
guidata(hObject,hand);

function Check_LeClaire_Callback(hObject, eventdata, hand)

function Output=imrotate2(A,deg,method,croppage)
%Find the midpoint
if(deg > 155)
    midx = ceil((size(A,1))/2);
    midy = ceil((size(A,2))/2);
else
    midx = ceil((size(A,2))/2);
    midy = ceil((size(A,1))/2);
end

[y,x] = meshgrid(1:size(A,2), 1:size(A,1));
[t,r] = cart2pol(x-midx,y-midy);
t1 = rad2deg(t)+deg;

%Convert from degree to radians
t = degtorad(t1);

%Convert to Cartesian Co-ordinates
[x,y] = pol2cart(t,r);

%Add the mid points to the new co-ordinates
tempx = round(x+midx);
tempy = round(y+midy);

if ( min(tempx(:)) < 0 )
    newx = max(tempx(:))+abs(min(tempx(:)))+1;
    tempx = tempx+abs(min(tempx(:)))+1;
else
    newx = max(tempx(:));
end
if newx<(size(A,1))
    newx=(size(A,1));
end
if( min(tempy( : )) < 0 )
    newy = max(tempy(:))+abs(min(tempy(:)))+1;
    tempy = tempy+abs(min(tempy(:)))+1;
else
    newy = max(tempy(:));
end
if newy<(size(A,2))
    newy=(size(A,2));
end
tempy(tempy==0) = 1;
tempx(tempx==0) = 1;

C = uint8(zeros([newx newy]));


for i = 1:size(A,1)
    for j = 1:size(A,2)
        C(tempx(i,j),tempy(i,j)) = A(i,j);
        
    end
    
end
Output = C;
%FILL THE HOLES OR GAPS-NEAREST NEIGHBOR
for i = 2:size(C,1)-1
    for j = 2:size(C,2)-1
        
        temp = C(i-1:i+1,j-1:j+1);
        if(temp(5)==0&&sum(temp(:))~=0)
            pt = find(temp~=0);
            
            [~,pos] = sort(abs(pt-5));
            Output(i,j) = temp(pt(pos(1)));
            
        end
        
    end
end

if strcmp(croppage,'crop')
    [a1,b1]=size(A);
    [a2,b2]=size(Output);
    aa=round((a2-a1)/2);
    bb=round((b2-b1)/2);
    try
        Output=Output(1+aa:aa+a1,1+bb:bb+b1);
    catch
        [1+aa,aa+a1,1+bb,bb+b1]
        keyboard
    end
end

function popupmenu6_Callback(hObject, eventdata, hand)
function popupmenu6_CreateFcn(hObject, eventdata, hand)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% hand    empty - hand not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ProfileMode_Callback(hObject, eventdata, hand)

set(hand.BoundCondMode,'enable','on')
switch get(hand.ProfileMode,'Value')
    case 1 %Crank
        Enable2='off';
        set(hand.t2,'enable','off')
        set(hand.k2_text,'string','k2 [m/s]')
        set(hand.InterfaceDepth,'visible','off')
    case 2 %Back-Crank analytical
        Enable2='off';
        set(hand.t2,'enable','on')
        set(hand.k2_text,'string','k2 [m/s]')
        set(hand.InterfaceDepth,'visible','off')
        set(hand.BoundCondMode,'value',1)
        set(hand.BoundCondMode,'enable','off')
        set(hand.MirrorPlane,'Visible','off');
        set(hand.Plane_text,'Visible','off');
    case 3 %Back-Crank numerical
        Enable2='on';
        set(hand.t2,'enable','on')
        set(hand.k2_text,'string','k2 [m/s]')
        set(hand.InterfaceDepth,'visible','off')
    case 4 %Interfacial
        Enable2='on';
        set(hand.k2_text,'string','r [s/m]')
        set(hand.k2,'enable','on')
        set(hand.k2,'string','1e9')
        set(hand.k2slider,'enable','on')
        set(hand.k2FitCheck,'enable','on')
        set(hand.InterfaceDepth,'visible','on')
    case 5 %LeClaire
        Enable2='on';
end
set(hand.D2,'enable',Enable2)
set(hand.D2slider,'enable',Enable2)
set(hand.D2FitCheck,'enable',Enable2)
set(hand.k2,'enable',Enable2)
set(hand.k2slider,'enable',Enable2)
set(hand.k2FitCheck,'enable',Enable2)
function ProfileMode_CreateFcn(hObject, eventdata, hand)
% hObject    handle to ProfileMode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% hand    empty - hand not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function InterfaceDepth_Callback(hObject, eventdata, hand)
function InterfaceDepth_CreateFcn(hObject, eventdata, hand)
% hObject    handle to InterfaceDepth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% hand    empty - hand not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
