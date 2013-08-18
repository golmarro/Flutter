function exportImage( name, fig )
%EXPORTIMAGE Summary of this function goes here
%   Detailed explanation goes here

if ~exist('fig','var')
    fig = gcf;
end

%set(gcf,'Units','centimeters','Position',[0, 0, 14, 8]);
%set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', [14 8]);


filePath = ['c:\Flutter\doc\Ilustracje\' , name];
% Zapis do eps nie uwzglednia w ogole PaperSize ani Position
% print('-depsc2',filePath);
saveas(fig,filePath,'fig')

end

