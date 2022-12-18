function [line1MP line2MP wallMP line1 line2]=safeLineSegCreator(map,distPt2Wall,bloatFactor)
    hold all
%     map=[5 5 7 7];
    unitVec = [(map(1)-map(3)) (map(2)-map(4))];
    wallLen=norm(unitVec);
    unitVec = unitVec/wallLen;
    normVec = [unitVec(2) -unitVec(1)]; % noraml vector to the line
%     plot([map(1) map(3)],[map(2) map(4)])
    hold on
    wallMP=[(map(1)+map(3))/2 (map(2)+map(4))/2];
%     k=bloatFactor+safetyDist;
%     k=1.2;
    line1MP=[wallMP(1)+distPt2Wall*normVec(1) wallMP(2)+distPt2Wall*normVec(2)];
    line2MP=[wallMP(1)-distPt2Wall*normVec(1) wallMP(2)-distPt2Wall*normVec(2)];
 
    safeLineHalfLen=(wallLen/2)-bloatFactor;
    %creating line seg 1
    line1pt1=[line1MP(1)+safeLineHalfLen*unitVec(1) line1MP(2)+safeLineHalfLen*unitVec(2)];
    line1pt2=[line1MP(1)-safeLineHalfLen*unitVec(1) line1MP(2)-safeLineHalfLen*unitVec(2)];
    line1=[line1pt1(1) line1pt1(2) line1pt2(1) line1pt2(2)];
    
    %creating line seg 2
    line2pt1=[line2MP(1)+safeLineHalfLen*unitVec(1) line2MP(2)+safeLineHalfLen*unitVec(2)];
    line2pt2=[line2MP(1)-safeLineHalfLen*unitVec(1) line2MP(2)-safeLineHalfLen*unitVec(2)];
    line2=[line2pt1(1) line2pt1(2) line2pt2(1) line2pt2(2)];    
    
%     plot([midpoint(1) midpoint(1)+unitVec(1)],[midpoint(2) midpoint(2)+unitVec(2)],'k')
%     hold on
%     plot([midpoint(1) midpoint(1)+normVec(1)],[midpoint(2) midpoint(2)+normVec(2)])
%     hold on

%     plot(DP1(1),DP1(2),'r*')
%     plot(DP2(1),DP2(2),'k*')
    
%     axis equal
end