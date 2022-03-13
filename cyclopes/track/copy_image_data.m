function ReferenceImage = copy_image_data(ReferenceImage,CurrentImage,WarpedImage)
ReferenceImage.I = CurrentImage.I;
ReferenceImage.polygon = WarpedImage.polygon;
ReferenceImage.index = WarpedImage.index;
ReferenceImage.Mask = WarpedImage.Mask;
end

