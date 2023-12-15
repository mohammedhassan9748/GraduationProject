# models.py
from django.db import models

# Employees Schema
class Employee(models.Model):
    EmployeeID = models.BigIntegerField(primary_key=True)
    EmployeeFirstName = models.CharField(max_length=50, null=False)
    EmployeeLastName = models.CharField(max_length=50, null=False)
    EmployeeUsername = models.CharField(max_length=50, null=False, unique=True)
    EmployeePassword = models.CharField(max_length=50, null=False)
    EmployeeRole = models.CharField(max_length=50, null=False)
    EmployeeEmail = models.CharField(max_length=70, null=False, unique=True)
    EmployeePhone = models.CharField(max_length=11, null=False, unique=True)
    EmployeeType = models.CharField(max_length=8, null=False,unique=True)

    class Meta:
        db_table = 'Employees'

# Inventory Schema
class Station(models.Model):
    StationID = models.BigIntegerField(primary_key=True)
    StationName = models.CharField(max_length=50, null=False, unique=True)

    class Meta:
        db_table = 'Stations'

class PackagesCategory(models.Model):
    PackageCategoryID = models.BigIntegerField(primary_key=True)
    PackageCategory = models.CharField(max_length=50, null=False, unique=True)

    class Meta:
        db_table = 'PackagesCategory'

class Package(models.Model):
    PackageID = models.BigIntegerField(primary_key=True)
    PackageName = models.CharField(max_length=100, null=False)
    PackageWeight = models.DecimalField(max_digits=4, decimal_places=2, null=False)
    PackageQRCode = models.CharField(max_length=200, null=False, unique=True)
    Station = models.ForeignKey(Station, on_delete=models.CASCADE, null=False)
    PackageCategory = models.ForeignKey(PackagesCategory, on_delete=models.CASCADE, null=False)

    class Meta:
        db_table = 'Packages'

class StorageLocation(models.Model):
    StorageLocationID = models.BigIntegerField(primary_key=True)
    Alias = models.CharField(max_length=50, null=False, unique=True)
    Package = models.ForeignKey(Package, on_delete=models.SET_NULL, null=True)

    class Meta:
        db_table = 'StorageLocations'

class Task(models.Model):
    TaskID = models.IntegerField(primary_key=True)
    Package = models.ForeignKey(Package, on_delete=models.SET_NULL, null=True)
    SourceLocation = models.CharField(max_length=255, null=False)
    DestinationLocation = models.CharField(max_length=255, null=False)
    TaskStatus = models.CharField(max_length=10, null=False)

    class Meta:
        db_table = 'Task'

class Robot(models.Model):
    RobotID = models.IntegerField(primary_key=True)
    RobotStatus = models.CharField(max_length=12, null=False)
    BatteryLevel = models.IntegerField(null=False)
    Task = models.ForeignKey(Task, on_delete=models.CASCADE)

    class Meta:
        db_table = 'Robot'

class Logs(models.Model):
    LogID = models.BigIntegerField(primary_key=True)
    Employee = models.ForeignKey(Employee, on_delete=models.CASCADE)
    LogTime = models.DateTimeField(null=False)
    LogAction = models.CharField(max_length=1024, null=False)

    class Meta:
        db_table = 'Logs'

