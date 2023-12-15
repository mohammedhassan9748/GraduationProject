from django.shortcuts import render, redirect
from django.contrib.auth import authenticate, login
from . import models
from decimal import Decimal

# Create your views here.
def Home(request):
    return render(request,'WAPP/login_options.html')

def Login_Employee(request):
    # Check if the form is submitted
    if request.method == 'POST':
        # Get the entered username and password from the form
        username = request.POST['username']
        password = request.POST['password']
        # Retrieve all employees
        for employee in models.Employee.objects.all():
            if employee.EmployeeUsername == username and employee.EmployeePassword == password:
                # Redirect to the desired page after successful login
                return redirect('Employee_Options')  # Update with your desired URL name
    return render(request,'WAPP/login_employee.html')
            
def Employee_Options(request):
    if request.method == 'POST':
        selected_option = request.POST.get('option')
        # Handle the selected option and redirect accordingly
        if selected_option == 'inventory_data_overview':
            return redirect('Inventory_data_overview')
        elif selected_option == 'manual_control':
            return redirect('Manual_Control')
        elif selected_option == 'amr_control':
            return redirect('AMR_Control')
    # If the form is not submitted or if there's an invalid option, you can render the same page or handle it as needed.
    return render(request, 'WAPP/employee_options.html')

def Data_Overview(request):
    if request.method == 'POST':
        option = request.POST.get('option','Packages')  # Default to 'Packages' if not provided
        if option == 'Packages':
            data = models.Package.objects.all()
        elif option == 'Storage Locations':
            data = models.StorageLocation.objects.all()
        else:
            # Handle invalid option gracefully
            data = []
        context = {'AllData': data, 'option': option}
        return render(request, 'WAPP/data_overview_packages.html', context)
    else:
        return render(request, 'WAPP/data_overview_packages.html') # Handle the initial refresh GET
    
    

def Manual_Control(request):

    PackagesData = models.Package.objects.all()
    PackagesIDs = [PackagesData[i].PackageID for i in range(len(PackagesData))]

    StorageLocationData = models.StorageLocation.objects.all()
    PackagesInStorageLocs = [StorageLocationData[i].Package for i in range(len(StorageLocationData))]
    StorageLocsIDs = [StorageLocationData[i].StorageLocationID for i in range(len(StorageLocationData))]

    EmptyPackageID = 0
    EmptyPackageID_Done = False
    for index in range(len(PackagesIDs)):
        if((index+1) != PackagesIDs[index]):
            EmptyPackageID = index+1
            EmptyPackageID_Done = True
            break
    if EmptyPackageID_Done== False:
        EmptyPackageID = len(PackagesIDs)+1
    
    context = {'NewPackageID': EmptyPackageID, 'stations':models.Station.objects.all(), 'categories': models.PackagesCategory.objects.all()}

    if request.method == 'POST':
        PackageName = request.POST['PackageName']
        PackageWeight = request.POST['PackageWeight']
        PackageQRCode = request.POST['PackageQRCode']
        Station = request.POST['Station']
        PackageCategory = request.POST['PackageCategory']
        # Create a new package instance
        new_package = models.Package(
            PackageID = EmptyPackageID,
            PackageName = PackageName,
            PackageWeight = round(float(PackageWeight), 1),
            PackageQRCode = PackageQRCode,
        )
        # Save the package to the database
        new_package.Station = models.Station.objects.get(StationName=Station)
        new_package.PackageCategory = models.PackagesCategory.objects.get(PackageCategory=PackageCategory)
        new_package.save()
        
        if Station == 'Storage':
            if PackageCategory == 'Electronics':
                for StorageLocsID in StorageLocsIDs:
                    if PackagesInStorageLocs[StorageLocsID] == None:
                        StorageLocationData[StorageLocsID].Package = PackagesData[len(PackagesData)-1]
                        StorageLocationData.save()
                        break

    return render(request, 'WAPP/manual_control.html',context)        
