function ReferenceImage = copy_image_data(ReferenceImage,CurrentImage)
ReferenceImage.I = CurrentImage.I;
ReferenceImage.polygon = CurrentImage.polygon;
ReferenceImage.index = CurrentImage.index;
ReferenceImage.Mask = CurrentImage.Mask;
end

