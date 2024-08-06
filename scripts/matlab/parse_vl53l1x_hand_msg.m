%% for demo
% jsonData = fileread('../../data/vl53l1x_proto_test/vl53l1x_proto_test.json');
jsonData = fileread('../../logs/hand_msg_00.json');

msgSet = jsondecode(jsonData);

dataMsgSet = {};

for i = 1:numel(msgSet)
    if isfield(msgSet(i), 'dataWrapper') && isfield(msgSet(i).dataWrapper, 'dataMsgs')
        dataMsgSet = [dataMsgSet; {msgSet(i)}];
    end
end

% Initialize a structure to store the data
dataStruct = struct();

% Iterate through msg.dataWrapper.dataMsgs.source and msg.chipType
for i = 1:numel(dataMsgSet)
    chipType = dataMsgSet{i}.chipType;
    for j = 1:numel(dataMsgSet{i}.dataWrapper.dataMsgs)
        dataMsg = dataMsgSet{i}.dataWrapper.dataMsgs(j);
        
        % Check if hasTimeStamp field exists
        if isfield(dataMsg, 'hasTimeStamp')
            warning('Message contains a timestamp field, further timestamps will be generated on the PC side.');
        else
            % Process timestamps
            timestamps = str2double(dataMsg.timestamps) / 1e6;  % Convert to seconds
            
            % Extract source and data
            source = char(dataMsg.source);
            data = dataMsg.data;
            
            % Store data into dataStruct
            if ~isfield(dataStruct, chipType)
                dataStruct.(chipType) = struct();
            end
            
            if ~isfield(dataStruct.(chipType), source)
                dataStruct.(chipType).(source) = struct('timestamps', [], 'data', []);
            end
            
            % Merge data
            dataStruct.(chipType).(source).timestamps = [dataStruct.(chipType).(source).timestamps; timestamps];
            dataStruct.(chipType).(source).data = [dataStruct.(chipType).(source).data; data];
        end
    end
end

% Create a container for figure handles
figure_handles = containers.Map();

% Iterate through each chipType to plot
chipTypes = fieldnames(dataStruct);
for i = 1:numel(chipTypes)
    chipType = chipTypes{i};
    
    % Create a figure
    figure_handles(chipType) = figure('Name', chipType, 'NumberTitle', 'off');
    hold on;
    
    % Get sources
    sources = fieldnames(dataStruct.(chipType));
    
    % Iterate through each source and plot
    for j = 1:numel(sources)
        source = sources{j};
        timestamps = dataStruct.(chipType).(source).timestamps;
        data = dataStruct.(chipType).(source).data;
        
        % Plot the data
        plot(timestamps, data, 'DisplayName', source);
    end
    
    % Set figure title and legend
    title(['Data for ', chipType]);
    legend;
    hold off;
end

% for i = 1:numel(dataMsgSet)
%     chipType = dataMsgSet{i}.chipType;
%     for j = 1:numel(dataMsgSet{i}.dataWrapper.dataMsgs)
%         if isfield(dataMsgSet{i}.dataWrapper.dataMsgs(j), 'source')
%             sources{end+1} = char(dataMsgSet{i}.dataWrapper.dataMsgs(j).source);
%              chipTypes{end+1} = chipType;
%         end
%     end
% end

% unique_sources = unique(sources);
% unique_chipTypes = unique(chipTypes);

% disp('data sources comes from: ')
% disp(unique_sources);

% disp('Unique chipTypes: ')
% disp(unique_chipTypes);

% figure_handles = containers.Map();

% for i = 1:numel(unique_chipTypes)
%     chipType = unique_chipTypes{i};
    
%     % 查找屬於當前 chipType 的 sources
%     idx = strcmp(chipTypes, chipType);
%     chipType_sources = sources(idx);
%     unique_chipType_sources = unique(chipType_sources);
    
%     % 創建圖形
%     figure_handles(chipType) = figure('Name', chipType, 'NumberTitle', 'off');
%     hold on;
    
%     % 添加圖例
%     for j = 1:numel(unique_chipType_sources)
%         legend_entry = unique_chipType_sources{j};
%         % 此處使用隨機數據繪圖，您需要根據實際數據進行替換
%         plot(rand(10,1), 'DisplayName', legend_entry);
%     end
    
%     title(['Data for ', chipType]);
%     legend;
%     hold off;
% end