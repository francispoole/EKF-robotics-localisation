% this function unwraps panoramic images and then finds the skyline and
% then gets some data
%
% it takes inputs fn which is the filename of the movie file that the data
% is stored in and is which is the index of each frame to be unwrapped
%
% example unwrappingFitting('nest1_trial.MP4',111:5:181)
%
% it currently expects each unwrapped frame of the data to be saved as a 
% mat file. This is because if youhave to use mmread to read data, it can
% be v slow for long files so it's better to load each frame and save it
% once (see AvisToMats2012.m for an example of this).
%
% however if you have version of matlab that's happy reading in your video
% data then change the lines where files are loaded in to read from it
% (currently lines 44 60 148) marked with **READ FROM MOVIE** as a comment

function unwrappingFitting(fn,is)

FitCircleAndUnwrap(fn,is,r1,r2)
% s=dir('*UnW.mat');
% SkyLine_Hand(s)
% GetSkylinesFromBinary


function FitCircleAndUnwrap(fn,is)
% badstart=34:30:2300;
% goodstart=318:30:2178;

goodstart=is;
todo=zeros(1,2339);
start_p=goodstart(1);%318;
end_p=goodstart(end);%2208;
nfr=2339
todo(1:start_p-1)=-1;
todo(end_p:nfr)=-1;
for i=goodstart
    todo(i:i+15)=1;
end
col='w';
cols=['w';'y';'r'];

% fit circle to first image: **READ FROM MOVIE**
load([fn(1:end-4) int2str(start_p) '.mat'])

% c=[630.9016  348.1678];rad=355
% [c,rad]=AdjustCircle(im,c,rad);
if(~isfile('tempcent.mat'))
    [cent,radius] = FitCircle2(im);
    save tempcent cent radius
    
    % test some images
    for i=goodstart(1:1:end)

        %     im2=MyAviRead(fn,i);
        %     subplot(2,1,1)
        %     imagesc(im2)
        %     subplot(2,1,2)
        
        % **READ FROM MOVIE**
        load([fn(1:end-4) int2str(i) '.mat'])
        PlotImRings(im,cent,radius,cols);
        title(int2str(i) )
        inp=input('any key to continue');
%         [c(i,:),rad(i)]=AdjustCircle(im,cent,radius,int2str(i));
    end
    
    save tempcent cent radius
else
    
    load tempcent
%     h=PlotImRings(im,cent,[radius;inn;out],cols);
%     pause
end

% 

% patt=[zeros(1,14) ones(1,16)];


% is=300:50:500;
% goods=ones(size(is))*-1;
% for i=is
% %     load([fn(1:end-4) int2str(i) '.mat'])
%     MyAviRead(fn,i);
%     imagesc(im)
%     inp=input('0 if bad');
%     if(isequal(inp,0))
%         goods(i)=0;
%     else
%         goods(i)=1;
%     end
% end
% 

% enter some inner and outer ring limits
if(~exist('inn'))
    h=PlotImRings(im,cent,radius,cols);
    hold on;
    [inn]=SetInOutRing(cent,'click inner ring',cols(2,:));
    hold off
    h=PlotImRings(im,cent,[radius;inn],cols);
    hold on;
    out=0;
    while(out<=inn)
        [out]=SetInOutRing(cent,'click outer ring',cols(3,:));
    end
    % specify how many degrees (roughly) this is
    ndeg=input('enter FOV (degs) between rings:  ');
    save tempcent inn out ndeg -append
end

% generate a meshgrid
% this is the nuimber of points per radius
np=90;
p=2*pi/np;

% these are the thetas: depending on the camera may need to change this
% ts=pi:-p:(-pi+p);
ts=0:-p:-(2*pi-p);
start_t=pi/2;
ts=ts+start_t;
d=(out-inn)/ndeg;
rads=(inn+d):d:(out);
% 
xM=[];yM=[];
for r=rads
    [xs,ys]=pol2cart(ts,r);
    xM=[xM; xs];
    yM=[yM; ys];
end
xM=[xM+cent(1)];
yM=[yM+cent(2)];
save('tempcent.mat','xM','yM','-append');%,'subsampfact','cent','');
% 
% % unwrap the images and save them
si=size(im);
[X,Y]=meshgrid(1:si(2),1:si(1));
hold off

is=goodstart;%start_p:15:end_p;
for i=is
    outf=[fn(1:end-4) int2str(i) 'UnW4.mat'];
    if(~isfile(outf))(todo(i)==1)
        i
        
        %  **READ FROM MOVIE**
        load([fn(1:end-4) int2str(i) '.mat'])
        unw_bw=interp2(X,Y,rgb2gray(im),xM,yM,'*cubic');
        for j=1:3
            unw_im(:,:,j)=interp2(X,Y,im(:,:,j),xM,yM,'*cubic');
        end
        imagesc(uint8(unw_im)),axis equal
        save(outf,'unw_im','unw_bw')
    end
end

function GetSkylinesFromBinary
s=dir('*Binary.mat');
skyc=200;
for i=1:length(s)
    [il,ilhe,bina,skyl_hi(i,:),skyl_lo(i,:),skysum,skysumh,skysumb,vals]= ...
        binim(s(i).name,skyc,1);
end
keyboard

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sub functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% this ftn processes an image into skyline
function[il,ilhe,bina,skyl_hi,skyl_lo,skysum,skysumh,skysumb,vals]= ...
    binim(fn,skyc,pl)

load(fn);
wid=size(newim,2);

bina(end,:)=1;
bina=double(bwfill(bina,'holes'));
bl=bwlabel(bina);
S=regionprops(bl,'Area');
[m,ind]=max([S.Area]);
% leave only the biggest object
bina=bl==ind;

% fill in any holes below 'horizon' at sides of image
rsum=sum(bina,2);
full_1=find(rsum==wid,1,'first');
bina(full_1:end,[1 wid])=1;
bina=double(bwfill(bina,'holes'));

% now cualculate a hi and low skyline
for i=1:wid
    skyl_lo(i)=double(find([0;bina(:,i)]==0,1,'last'));
    skyl_hi(i)=double(find([bina(:,i)]==1,1,'first'));
end

%plot im and sky
if pl
    imagesc(newimall)
    hold on;
    plot(1:wid,skyl_lo,'r',1:wid,skyl_hi,'c','LineWidth',1.5)
    axis image
    hold off
end

% histeg'ed image and sky only
imhe=double(histeq(newim));
skyonly=double(~bina);

% this gets the image with sky being 0
il=newim.*bina;
ilhe=imhe.*bina;

% get a sum of the amount of 'ground' from the varoius manipulations
skysum=sum(il);
skysumh=sum(ilhe);
skysumb=sum(bina);

% this then makes the sky a uniform colour, skyc
il=il+skyc*skyonly;
ilhe=ilhe+skyc*skyonly;

% this bit seems to be emphasising sky: not entirely sure why
sky=double(newim).*skyonly;
skyh=double(imhe).*skyonly;
sky2=il*1e3+sky;
skyh2=il*1e3+skyh;

% get some vals: again was poss useful but might not still be
vals=[max(sky(:)) min(sky2(:)) max(skyh(:))  min(skyh2(:))];






function[il,ilhe,bina,skyl_hi,skyl_lo,skysum,skysumh,skysumb,vals]= ...
    binimAll(newim,imhe,imrgb,skyc,fn)

wid=size(newim,2);
il=double(newim);
fnew=[fn(1:end-8) '_Binary.mat'];
load(fnew);

bina(end,:)=1;
bina=double(bwfill(bina,'holes'));

il=double(newim).*double(bina);
ilhe=double(imhe).*double(bina);
skysum=sum(il);
skysumh=sum(ilhe);
skysumb=sum(double(bina));

sky=double(newim).*double(~bina);
skyh=double(imhe).*double(~bina);
sky2=il*1e3+sky;
skyh2=il*1e3+skyh;

vals=[max(sky(:)) min(sky2(:)) max(skyh(:))  min(skyh2(:))];

il=il+skyc*double(~bina);
ilhe=ilhe+skyc*double(~bina);

for i=1:wid
    skyl_lo(i)=double(find([0;bina(:,i)]==0,1,'last'));
    skyl_hi(i)=double(find([bina(:,i)]==1,1,'first'));
end
imagesc(imrgb),hold on;
plot(1:wid,skyl_lo,'r',1:wid,skyl_hi,'c','LineWidth',1.5)
axis image, hold off


% plot things
function[h]=PlotImRings(im,cent,rads,cols)
imagesc(im)
hold on;
h=MyCircle(repmat(cent,size(rads,1),1),rads,cols);
h=[h(:,1);plot(cent(1),cent(2),[cols(1,:) 'x'],'MarkerSize',10)];
axis equal
hold off



% enter some inner and outer ring limits
function[rad]=SetInOutRing(c,str,col)
rad=-1;
h=[];
while 1
    title(str)
    [x,y,b]=ginput(1);
    if(isempty(x)&&(rad>0))
        break;
    else
        delete(h);
        rad=CartDist(c,[x y]);
        h=MyCircle(c,rad,col);
    end
end

function [c,rad] = FitCircle2(im,sp,axw)
clf
imagesc(im);
if(nargin==3) axis(axw); end;
axis equal;
hold on;
if(nargin<2)
    [c,rad]=GetStartPoint;
else
    c=sp(1:2);
    rad=sp(3);
    t=pi/2;
end
MyCircleLocal(c(1),c(2),rad,'r');
plot(c(1),c(2),'r.');
hold off;

[c,rad]=AdjustCircle(im,c,rad);

function[c,rad]=AdjustCircle(im,c,rad,str)
imagesc(im)
axis equal
if(nargin>3)
    xlabel(str);
end
hold on;
col='w';
if(nargin>2)
    h=MyCircle(c,rad,col);
    h=[h;plot(c(1),c(2),[col 'x'],'MarkerSize',10)];
else
    h=[];    
end
pwadd=0.25;
while 1
    r=rad*1.2;
    axis([c(1)-r c(1)+r c(2)-r c(2)+r])
    title('click position; cursors move x; [ or ]: change radius x; x set x; return end')
    [x,y,b]=ginput(1);
    if(isempty(x))
        delete(h);
        break;
    elseif(b==30) % up cursor
        c(2)=c(2)-pwadd;
    elseif(b==31) % down cursor
        c(2)=c(2)+pwadd;
    elseif(b==28) % left cursor
        c(1)=c(1)-pwadd;
    elseif(b==29) % right cursor
        c(1)=c(1)+pwadd;
    elseif(b==93) % right bracket ]
        rad=rad+pwadd;
    elseif(b==91) % left bracket [
        rad=max(rad-pwadd,1);
    elseif(b==120) % x
        pwadd=input(['x = ' num2str(pwadd) '; enter new value: ']);
    else
        c=[x y];
    end
    delete(h);
    h=MyCircle(c,rad,col);
    h=[h;plot(c(1),c(2),[col 'x'],'MarkerSize',10)];
end
hold off;


function[c,rad]=GetStartPoint
title('Click 7 points on the radius')
[x,y]=ginput(7);
% solve
abc=pinv([x,y,ones(size(x))])*(x.^2+y.^2);

c=[abc(1)/2,abc(2)/2];
rad=mean(sqrt((x-c(1)).^2+(y-c(2)).^2));

% rad=sqrt((abc(1)/2)^2+(abc(2)/2)^2-abc(3));

function im1=resizeImage(im,acuity)
% acuity is in degrees i.e. acuity=4 -> each pixel is 4 degrees,
% acuity = 0.2 ->5 pixels /degree
if nargin==1
    acuity=1;
end
% check whether colour or black and white
[a,b,c]=size(im);
if c==1
    % black and white
    B=double(im);
    [h,w]=size(im);
    block_sz=acuity*round(w/360);

    % make sure the dimensions are whole numbers

    max_h=floor(h/block_sz)*block_sz;
    max_w=floor(w/block_sz)*block_sz;

    B=B(1:max_h,1:max_w);

    fun=@(x) mean(x(:));

    im1=blkproc(B,[block_sz block_sz],fun);
else
    %colour
    R=double(im(:,:,1));
    B=double(im(:,:,2));
    G=double(im(:,:,3));
    [h,w]=size(R);
    block_sz=acuity*round(w/360);

    % make sure the dimensions are whole numbers

    max_h=floor(h/block_sz)*block_sz;
    max_w=floor(w/block_sz)*block_sz;

    R=R(1:max_h,1:max_w);
    B=B(1:max_h,1:max_w);
    G=G(1:max_h,1:max_w);
    % anonymous function
    fun=@(x) mean(x(:));

    im1(:,:,1)=uint8(blkproc(R,[block_sz block_sz],fun));
    im1(:,:,2)=uint8(blkproc(B,[block_sz block_sz],fun));
    im1(:,:,3)=uint8(blkproc(G,[block_sz block_sz],fun));
end

function[Out,fn]=isfile(fname,DName)

fname=char(fname);
if(nargin<2)
	s=dir(fname);
	if(length(s)==0)
		Out=0;
		fn=[];
	else
		Out=1;
		fn=char(s.name);
	end
else
    if(computer=='PCWIN') Sep='/';
    else Sep=':';
    end
    dstr=[DName Sep fname];
	s=dir(dstr);
	if(length(s)==0)
		Out=0;
		fn=[];
	else
		Out=1;
		Ds=[];
		for i=1:length(s)
			Ds=[Ds; [DName Sep]];
		end		
		fn=[Ds [char(s.name)]];
	end
end


% function[lhdl] = MyCircleLocal(x,y,Rad,col,NumPts,fillc)
%
% Function draws circles of radius Rad(i) at x(:,i),y(:,i). 
% col is colour, NumPts the number of points to draw the circle
% defaults are blue and 50 and fills it if fillc = 1 (default 0)
%
% if x is a 2D row vector, it uses x as position and other parameters 
% shift across one eg y is rad
%
% function returns ldhl, handles to the lines
function[lhdl] = MyCircleLocal(x,y,Rad,col,NumPts,fillc)
ho=ishold;
if(size(x,2)==2)
    if(nargin<5) 
        fillc=0;
    else
        fillc=NumPts;
    end
    if((nargin<4)||isempty(col)) 
        NumPts=50;
    else
        NumPts=col;
    end;
    if(nargin<3) 
        col = 'b';
    else
        col=Rad;
    end;
    Rad=y;
    y=x(:,2);
    x=x(:,1);
else
    if(nargin<6) 
        fillc=0; 
    end;
    if((nargin<5)||isempty(NumPts)) 
        NumPts=50; 
    end;
    if(nargin<4) 
        col = 'b'; 
    end;
end

Thetas=0:2*pi/NumPts:2*pi;
lhdl=zeros(length(Rad));
if(size(col,1)==1)
    for i=1:length(Rad)
        [Xs,Ys]=pol2cart(Thetas,Rad(i));
        if(fillc) 
            fill(Xs+x(i),Ys+y(i),col);
            hold on;
        end
        lhdl(i)=plot(Xs+x(i),Ys+y(i),'Color',col);
        % THIS ERROR LOOKS TO BE A VERSION CHANGE: BELOW FOR NEW VERSIONS
%         lhdl(i)=plot(Xs+x(i),Ys+y(i),col);
        hold on
    end
else
    for i=1:length(Rad)
        [Xs,Ys]=pol2cart(Thetas,Rad(i));
        if(fillc) 
            fill(Xs+x(i),Ys+y(i),col(i,:));
            hold on;
        end
        lhdl(i)=plot(Xs+x(i),Ys+y(i),'Color',col(i,:));
        % THIS ERROR LOOKS TO BE A VERSION CHANGE: BELOW FOR NEW VERSIONS
%         lhdl(i)=plot(Xs+x(i),Ys+y(i),col(i,:));
        hold on
    end
end
if(~ho) 
    hold off; 
end;