function Optimal_path = path_from_A_star(map)

Optimal_path = [];
num_p = size(map,1);

MAX_X=10;
MAX_Y=10;

%Define the 2D grid map array.
%Obstacle=-1, Target = 0, Start=1
MAP=2*(ones(MAX_X,MAX_Y));

%Initialize MAP with location of the target
xval=floor(map(num_p, 1));
yval=floor(map(num_p, 2));
xTarget=xval;
yTarget=yval;
MAP(xval,yval)=0;

%Initialize MAP with location of the obstacle
for i = 2: num_p-1
    xval=floor(map(i, 1));
    yval=floor(map(i, 2));
    MAP(xval,yval)=-1;
end 

%Initialize MAP with location of the start point
xval=floor(map(1, 1));
yval=floor(map(1, 2));
xStart=xval;
yStart=yval;
MAP(xval,yval)=1;

%start your code
OPEN=[];
CLOSED=[];
%Put all obstacles on the Closed list
k=1;%Dummy counter
for i=1:MAX_X
for j=1:MAX_Y
    if(MAP(i,j) == -1)
        CLOSED(k,1)=i; 
        CLOSED(k,2)=j; 
        k=k+1;
    end
end
end
CLOSED_COUNT=size(CLOSED,1);
%set the starting node as the first node
xNode=xval;
yNode=yval;
OPEN_COUNT=1;
path_cost=0;
goal_distance=distance(xNode,yNode,xTarget,yTarget);
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);
OPEN(OPEN_COUNT,1)=0;
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED(CLOSED_COUNT,1)=xNode;
CLOSED(CLOSED_COUNT,2)=yNode;
NoPath=1;

while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
    exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y);
    exp_count=size(exp_array,1);

    for i=1:exp_count
        flag=0;
        for j=1:OPEN_COUNT
            if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )
                OPEN(j,8)=min(OPEN(j,8),exp_array(i,5)); %#ok<*SAGROW>
                if OPEN(j,8)== exp_array(i,5)
                    %UPDATE PARENTS,gn,hn
                    OPEN(j,4)=xNode;
                    OPEN(j,5)=yNode;
                    OPEN(j,6)=exp_array(i,3);
                    OPEN(j,7)=exp_array(i,4);
                end;%End of minimum fn check
                flag=1;
            end;%End of node check
        end;%End of j for
        if flag == 0
            OPEN_COUNT = OPEN_COUNT+1;
            OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
        end;%End of insert new element into the OPEN list
    end;%End of i for
    %Find out the node with the smallest fn 
    index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
    if (index_min_node ~= -1)    
        %Set xNode and yNode to the node with minimum fn
        xNode=OPEN(index_min_node,2);
        yNode=OPEN(index_min_node,3);
        path_cost=OPEN(index_min_node,6);%Update the cost of reaching the parent node
        %Move the Node to list CLOSED
        CLOSED_COUNT=CLOSED_COUNT+1;
        CLOSED(CLOSED_COUNT,1)=xNode;
        CLOSED(CLOSED_COUNT,2)=yNode;
        OPEN(index_min_node,1)=0;
    else
        %No path exists to the Target!!
        NoPath=0;%Exits the loop!
    end;%End of index_min_node check
end;%End of While Loop

i=size(CLOSED,1);
xval=CLOSED(i,1);
yval=CLOSED(i,2);
i=1;
Optimal_path(i,1)=xval;
Optimal_path(i,2)=yval;
i=i+1;
if ((xval == xTarget) && (yval == yTarget))
    inode=0;
    %Traverse OPEN and determine the parent nodes
    parent_x=OPEN(node_index(OPEN,xval,yval),4);%node_index returns the index of the node
    parent_y=OPEN(node_index(OPEN,xval,yval),5);

    while( parent_x ~= xStart || parent_y ~= yStart)
           Optimal_path(i,1) = parent_x;
           Optimal_path(i,2) = parent_y;
           inode=node_index(OPEN,parent_x,parent_y);
           parent_x=OPEN(inode,4);%node_index returns the index of the node
           parent_y=OPEN(inode,5);
           i=i+1;
    end;
end
sz=size(Optimal_path,1);
opp=[];
for i=1:sz
    opp(i,1:2)=Optimal_path(sz+1-i,:);
    opp(i,3)=map(1,3);
end
Optimal_path=[xStart yStart map(1,3); opp];

end